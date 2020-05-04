##Copyright 2018 Thomas Paviot (tpaviot@gmail.com)
##
##This file is part of pythonOCC.
##
##pythonOCC is free software: you can redistribute it and/or modify
##it under the terms of the GNU Lesser General Public License as published by
##the Free Software Foundation, either version 3 of the License, or
##(at your option) any later version.
##
##pythonOCC is distributed in the hope that it will be useful,
##but WITHOUT ANY WARRANTY; without even the implied warranty of
##MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##GNU Lesser General Public License for more details.
##
##You should have received a copy of the GNU Lesser General Public License
##along with pythonOCC.  If not, see <http://www.gnu.org/licenses/>.

import os
import time
import uuid

from OCC import VERSION as OCC_VERSION
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.StlAPI import stlapi_Read, StlAPI_Writer
from OCC.Core.BRep import BRep_Builder
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Pnt2d
from OCC.Core.Bnd import Bnd_Box2d
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.IGESControl import IGESControl_Reader, IGESControl_Writer
from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCC.Core.Interface import Interface_Static_SetCVal
from OCC.Core.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.XCAFDoc import (XCAFDoc_DocumentTool_ShapeTool,
                              XCAFDoc_DocumentTool_ColorTool)
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.TDF import TDF_LabelSequence, TDF_Label
from OCC.Core.TCollection import TCollection_ExtendedString
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.Tesselator import ShapeTesselator

from OCC.Extend.TopologyUtils import (discretize_edge, get_sorted_hlr_edges,
                                      list_of_shapes_to_compound)
try:
    from OCC.Extend.MeshTools import optimize_mesh
    HAVE_MESH_TOOLS = True
except:
    HAVE_MESH_TOOLS = False

try:
    import svgwrite
    HAVE_SVGWRITE = True
except ImportError:
    HAVE_SVGWRITE = False

##########################
# Step import and export #
##########################
def read_step_file(filename, as_compound=True, verbosity=True):
    """ read the STEP file and returns a compound
    filename: the file path
    verbosity: optional, False by default.
    as_compound: True by default. If there are more than one shape at root,
    gather all shapes into one compound. Otherwise returns a list of shapes.
    """
    if not os.path.isfile(filename):
        raise FileNotFoundError("%s not found." % filename)

    step_reader = STEPControl_Reader()
    status = step_reader.ReadFile(filename)

    if status == IFSelect_RetDone:  # check status
        if verbosity:
            failsonly = False
            step_reader.PrintCheckLoad(failsonly, IFSelect_ItemsByEntity)
            step_reader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity)
        transfer_result = step_reader.TransferRoots()
        if not transfer_result:
            raise AssertionError("Transfer failed.")
        _nbs = step_reader.NbShapes()
        if _nbs == 0:
            raise AssertionError("No shape to transfer.")
        elif _nbs == 1:  # most cases
            return step_reader.Shape(1)
        elif _nbs > 1:
            print("Number of shapes:", _nbs)
            shps = []
            # loop over root shapes
            for k in range(1, _nbs + 1):
                new_shp = step_reader.Shape(k)
                if not new_shp.IsNull():
                    shps.append(new_shp)
            if as_compound:
                compound, result = list_of_shapes_to_compound(shps)
                if not result:
                    print("Warning: all shapes were not added to the compound")
                return compound
            else:
                print("Warning, returns a list of shapes.")
                return shps
    else:
        raise AssertionError("Error: can't read file.")
    return None


def write_step_file(a_shape, filename, application_protocol="AP203"):
    """ exports a shape to a STEP file
    a_shape: the topods_shape to export (a compound, a solid etc.)
    filename: the filename
    application protocol: "AP203" or "AP214IS" or "AP242DIS"
    """
    # a few checks
    if a_shape.IsNull():
        raise AssertionError("Shape %s is null." % a_shape)
    if application_protocol not in ["AP203", "AP214IS", "AP242DIS"]:
        raise AssertionError("application_protocol must be either AP203 or AP214IS. You passed %s." % application_protocol)
    if os.path.isfile(filename):
        print("Warning: %s file already exists and will be replaced" % filename)
    # creates and initialise the step exporter
    step_writer = STEPControl_Writer()
    Interface_Static_SetCVal("write.step.schema", application_protocol)

    # transfer shapes and write file
    step_writer.Transfer(a_shape, STEPControl_AsIs)
    status = step_writer.Write(filename)

    if not status == IFSelect_RetDone:
        raise IOError("Error while writing shape to STEP file.")
    if not os.path.isfile(filename):
        raise IOError("File %s was not saved to filesystem." % filename)


def read_step_file_with_names_colors(filename):
    """ Returns list of tuples (topods_shape, label, color)
    Use OCAF.
    """
    if not os.path.isfile(filename):
        raise FileNotFoundError("%s not found." % filename)
    # the list:
    output_shapes = {}

    # create an handle to a document
    doc = TDocStd_Document(TCollection_ExtendedString("pythonocc-doc"))

    # Get root assembly
    shape_tool = XCAFDoc_DocumentTool_ShapeTool(doc.Main())
    color_tool = XCAFDoc_DocumentTool_ColorTool(doc.Main())
    #layer_tool = XCAFDoc_DocumentTool_LayerTool(doc.Main())
    #mat_tool = XCAFDoc_DocumentTool_MaterialTool(doc.Main())

    step_reader = STEPCAFControl_Reader()
    step_reader.SetColorMode(True)
    step_reader.SetLayerMode(True)
    step_reader.SetNameMode(True)
    step_reader.SetMatMode(True)
    step_reader.SetGDTMode(True)

    status = step_reader.ReadFile(filename)
    if status == IFSelect_RetDone:
        step_reader.Transfer(doc)

    locs = []

    def _get_sub_shapes(lab, loc):
        #global cnt, lvl
        #cnt += 1
        #print("\n[%d] level %d, handling LABEL %s\n" % (cnt, lvl, _get_label_name(lab)))
        #print()
        #print(lab.DumpToString())
        #print()
        #print("Is Assembly    :", shape_tool.IsAssembly(lab))
        #print("Is Free        :", shape_tool.IsFree(lab))
        #print("Is Shape       :", shape_tool.IsShape(lab))
        #print("Is Compound    :", shape_tool.IsCompound(lab))
        #print("Is Component   :", shape_tool.IsComponent(lab))
        #print("Is SimpleShape :", shape_tool.IsSimpleShape(lab))
        #print("Is Reference   :", shape_tool.IsReference(lab))

        #users = TDF_LabelSequence()
        #users_cnt = shape_tool.GetUsers(lab, users)
        #print("Nr Users       :", users_cnt)

        l_subss = TDF_LabelSequence()
        shape_tool.GetSubShapes(lab, l_subss)
        #print("Nb subshapes   :", l_subss.Length())
        l_comps = TDF_LabelSequence()
        shape_tool.GetComponents(lab, l_comps)
        #print("Nb components  :", l_comps.Length())
        #print()
        name = lab.GetLabelName()
        print("Name :", name)

        if shape_tool.IsAssembly(lab):
            l_c = TDF_LabelSequence()
            shape_tool.GetComponents(lab, l_c)
            for i in range(l_c.Length()):
                label = l_c.Value(i+1)
                if shape_tool.IsReference(label):
                    #print("\n########  reference label :", label)
                    label_reference = TDF_Label()
                    shape_tool.GetReferredShape(label, label_reference)
                    loc = shape_tool.GetLocation(label)
                    #print("    loc          :", loc)
                    #trans = loc.Transformation()
                    #print("    tran form    :", trans.Form())
                    #rot = trans.GetRotation()
                    #print("    rotation     :", rot)
                    #print("    X            :", rot.X())
                    #print("    Y            :", rot.Y())
                    #print("    Z            :", rot.Z())
                    #print("    W            :", rot.W())
                    #tran = trans.TranslationPart()
                    #print("    translation  :", tran)
                    #print("    X            :", tran.X())
                    #print("    Y            :", tran.Y())
                    #print("    Z            :", tran.Z())

                    locs.append(loc)
                    #print(">>>>")
                    #lvl += 1
                    _get_sub_shapes(label_reference, loc)
                    #lvl -= 1
                    #print("<<<<")
                    locs.pop()

        elif shape_tool.IsSimpleShape(lab):
            #print("\n########  simpleshape label :", lab)
            shape = shape_tool.GetShape(lab)
            #print("    all ass locs   :", locs)

            loc = TopLoc_Location()
            for l in locs:
                #print("    take loc       :", l)
                loc = loc.Multiplied(l)

            #trans = loc.Transformation()
            #print("    FINAL loc    :")
            #print("    tran form    :", trans.Form())
            #rot = trans.GetRotation()
            #print("    rotation     :", rot)
            #print("    X            :", rot.X())
            #print("    Y            :", rot.Y())
            #print("    Z            :", rot.Z())
            #print("    W            :", rot.W())
            #tran = trans.TranslationPart()
            #print("    translation  :", tran)
            #print("    X            :", tran.X())
            #print("    Y            :", tran.Y())
            #print("    Z            :", tran.Z())
            c = Quantity_Color(0.5, 0.5, 0.5, Quantity_TOC_RGB)  # default color
            colorSet = False
            if (color_tool.GetInstanceColor(shape, 0, c) or
                    color_tool.GetInstanceColor(shape, 1, c) or
                    color_tool.GetInstanceColor(shape, 2, c)):
                color_tool.SetInstanceColor(shape, 0, c)
                color_tool.SetInstanceColor(shape, 1, c)
                color_tool.SetInstanceColor(shape, 2, c)
                colorSet = True
                n = c.Name(c.Red(), c.Green(), c.Blue())
                print('    instance color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())

            if not colorSet:
                if (color_tool.GetColor(lab, 0, c) or
                        color_tool.GetColor(lab, 1, c) or
                        color_tool.GetColor(lab, 2, c)):

                    color_tool.SetInstanceColor(shape, 0, c)
                    color_tool.SetInstanceColor(shape, 1, c)
                    color_tool.SetInstanceColor(shape, 2, c)

                    n = c.Name(c.Red(), c.Green(), c.Blue())
                    print('    shape color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())

            shape_disp = BRepBuilderAPI_Transform(shape, loc.Transformation()).Shape()
            if not shape_disp in output_shapes:
                output_shapes[shape_disp] = [lab.GetLabelName(), c]
            for i in range(l_subss.Length()):
                lab_subs = l_subss.Value(i+1)
                #print("\n########  simpleshape subshape label :", lab)
                shape_sub = shape_tool.GetShape(lab_subs)

                c = Quantity_Color(0.5, 0.5, 0.5, Quantity_TOC_RGB)  # default color
                colorSet = False
                if (color_tool.GetInstanceColor(shape_sub, 0, c) or
                        color_tool.GetInstanceColor(shape_sub, 1, c) or
                        color_tool.GetInstanceColor(shape_sub, 2, c)):
                    color_tool.SetInstanceColor(shape_sub, 0, c)
                    color_tool.SetInstanceColor(shape_sub, 1, c)
                    color_tool.SetInstanceColor(shape_sub, 2, c)
                    colorSet = True
                    n = c.Name(c.Red(), c.Green(), c.Blue())
                    print('    instance color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())

                if not colorSet:
                    if (color_tool.GetColor(lab_subs, 0, c) or
                            color_tool.GetColor(lab_subs, 1, c) or
                            color_tool.GetColor(lab_subs, 2, c)):
                        color_tool.SetInstanceColor(shape, 0, c)
                        color_tool.SetInstanceColor(shape, 1, c)
                        color_tool.SetInstanceColor(shape, 2, c)

                        n = c.Name(c.Red(), c.Green(), c.Blue())
                        print('    shape color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())
                shape_to_disp = BRepBuilderAPI_Transform(shape_sub, loc.Transformation()).Shape()
                # position the subshape to display
                if not shape_to_disp in output_shapes:
                    output_shapes[shape_to_disp] = [lab_subs.GetLabelName(), c]


    def _get_shapes():
        labels = TDF_LabelSequence()
        shape_tool.GetFreeShapes(labels)
        #global cnt
        #cnt += 1

        print()
        print("Number of shapes at root :", labels.Length())
        print()
        for i in range(labels.Length()):
            root_item = labels.Value(i+1)
            _get_sub_shapes(root_item, None)
    _get_shapes()
    return output_shapes


#########################
# STL import and export #
#########################
def write_stl_file(a_shape, filename, mode="ascii", linear_deflection=0.9, angular_deflection=0.5):
    """ export the shape to a STL file
    Be careful, the shape first need to be explicitely meshed using BRepMesh_IncrementalMesh
    a_shape: the topods_shape to export
    filename: the filename
    mode: optional, "ascii" by default. Can either be "binary"
    linear_deflection: optional, default to 0.001. Lower, more occurate mesh
    angular_deflection: optional, default to 0.5. Lower, more accurate_mesh
    """
    if a_shape.IsNull():
        raise AssertionError("Shape is null.")
    if mode not in ["ascii", "binary"]:
        raise AssertionError("mode should be either ascii or binary")
    if os.path.isfile(filename):
        print("Warning: %s file already exists and will be replaced" % filename)
    # first mesh the shape
    mesh = BRepMesh_IncrementalMesh(a_shape, linear_deflection, False, angular_deflection, True)
    #mesh.SetDeflection(0.05)
    mesh.Perform()
    if not mesh.IsDone():
        raise AssertionError("Mesh is not done.")

    stl_exporter = StlAPI_Writer()
    if mode == "ascii":
        stl_exporter.SetASCIIMode(True)
    else:  # binary, just set the ASCII flag to False
        stl_exporter.SetASCIIMode(False)
    stl_exporter.Write(a_shape, filename)

    if not os.path.isfile(filename):
        raise IOError("File not written to disk.")


def read_stl_file(filename):
    """ opens a stl file, reads the content, and returns a BRep topods_shape object
    """
    if not os.path.isfile(filename):
        raise FileNotFoundError("%s not found." % filename)

    the_shape = TopoDS_Shape()
    stlapi_Read(the_shape, filename)

    if the_shape.IsNull():
        raise AssertionError("Shape is null.")

    return the_shape

######################
# IGES import/export #
######################
def read_iges_file(filename, return_as_shapes=False, verbosity=False, visible_only=False):
    """ read the IGES file and returns a compound
    filename: the file path
    return_as_shapes: optional, False by default. If True returns a list of shapes,
                      else returns a single compound
    verbosity: optionl, False by default.
    """
    if not os.path.isfile(filename):
        raise FileNotFoundError("%s not found." % filename)

    iges_reader = IGESControl_Reader()
    iges_reader.SetReadVisible(visible_only)
    status = iges_reader.ReadFile(filename)

    _shapes = []

    if status == IFSelect_RetDone:  # check status
        if verbosity:
            failsonly = False
            iges_reader.PrintCheckLoad(failsonly, IFSelect_ItemsByEntity)
            iges_reader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity)
        iges_reader.TransferRoots()
        nbr = iges_reader.NbRootsForTransfer()
        for _ in range(1, nbr+1):
            nbs = iges_reader.NbShapes()
            if nbs == 0:
                print("At least one shape in IGES cannot be transfered")
            elif nbr == 1 and nbs == 1:
                a_res_shape = iges_reader.Shape(1)
                if a_res_shape.IsNull():
                    print("At least one shape in IGES cannot be transferred")
                else:
                    _shapes.append(a_res_shape)
            else:
                for i in range(1, nbs+1):
                    a_shape = iges_reader.Shape(i)
                    if a_shape.IsNull():
                        print("At least one shape in STEP cannot be transferred")
                    else:
                        _shapes.append(a_shape)
    # if not return as shapes
    # create a compound and store all shapes
    if not return_as_shapes:
        builder = BRep_Builder()
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)
        for s in _shapes:
            builder.Add(compound, s)
        _shapes = compound
    return _shapes

def write_iges_file(a_shape, filename):
    """ exports a shape to a STEP file
    a_shape: the topods_shape to export (a compound, a solid etc.)
    filename: the filename
    application protocol: "AP203" or "AP214"
    """
    # a few checks
    if a_shape.IsNull():
        raise AssertionError("Shape is null.")
    if os.path.isfile(filename):
        print("Warning: %s file already exists and will be replaced" % filename)
    # creates and initialise the step exporter
    iges_writer = IGESControl_Writer()
    iges_writer.AddShape(a_shape)
    status = iges_writer.Write(filename)

    if status != IFSelect_RetDone:
        raise AssertionError("Not done.")
    if not os.path.isfile(filename):
        raise IOError("File not written to disk.")


##############
# SVG export #
##############
def edge_to_svg_polyline(topods_edge, tol=0.1, unit="mm"):
    """ Returns a svgwrite.Path for the edge, and the 2d bounding box
    """
    unit_factor = 1  # by default

    if unit == "mm":
        unit_factor = 1
    elif unit == "m":
        unit_factor = 1e3

    points_3d = discretize_edge(topods_edge, tol)
    points_2d = []
    box2d = Bnd_Box2d()

    for point in points_3d:
        # we tak only the first 2 coordinates (x and y, leave z)
        x_p = - point[0] * unit_factor
        y_p = point[1] * unit_factor
        box2d.Add(gp_Pnt2d(x_p, y_p))
        points_2d.append((x_p, y_p))

    return svgwrite.shapes.Polyline(points_2d, fill="none"), box2d

def export_shape_to_svg(shape, filename=None,
                        width=800, height=600, margin_left=10,
                        margin_top=30, export_hidden_edges=True,
                        location=gp_Pnt(0, 0, 0), direction=gp_Dir(1, 1, 1),
                        color="black",
                        line_width="1px",
                        unit="mm"):
    """ export a single shape to an svg file and/or string.
    shape: the TopoDS_Shape to export
    filename (optional): if provided, save to an svg file
    width, height (optional): integers, specify the canva size in pixels
    margin_left, margin_top (optional): integers, in pixel
    export_hidden_edges (optional): whether or not draw hidden edges using a dashed line
    location (optional): a gp_Pnt, the lookat
    direction (optional): to set up the projector direction
    color (optional), "default to "black".
    line_width (optional, default to 1): an integer
    """
    if shape.IsNull():
        raise AssertionError("shape is Null")

    if not HAVE_SVGWRITE:
        print("svg exporter not available because the svgwrite package is not installed.")
        print("please use '$ conda install -c conda-forge svgwrite'")
        return False

    # find all edges
    visible_edges, hidden_edges = get_sorted_hlr_edges(shape, position=location, direction=direction, export_hidden_edges=export_hidden_edges)

    # compute polylines for all edges
    # we compute a global 2d bounding box as well, to be able to compute
    # the scale factor and translation vector to apply to all 2d edges so that
    # they fit the svg canva
    global_2d_bounding_box = Bnd_Box2d()

    polylines = []
    for visible_edge in visible_edges:
        visible_svg_line, visible_edge_box2d = edge_to_svg_polyline(visible_edge, 0.1, unit)
        polylines.append(visible_svg_line)
        global_2d_bounding_box.Add(visible_edge_box2d)
    if export_hidden_edges:
        for hidden_edge in hidden_edges:
            hidden_svg_line, hidden_edge_box2d = edge_to_svg_polyline(hidden_edge, 0.1, unit)
            # hidden lines are dashed style
            hidden_svg_line.dasharray([5, 5])
            polylines.append(hidden_svg_line)
            global_2d_bounding_box.Add(hidden_edge_box2d)

    # translate and scale polylines

    # first compute shape translation and scale according to size/margins
    x_min, y_min, x_max, y_max = global_2d_bounding_box.Get()
    bb2d_width = x_max - x_min
    bb2d_height = y_max - y_min

    # build the svg drawing
    dwg = svgwrite.Drawing(filename, (width, height), debug=True)
    # adjust the view box so that the lines fit then svg canvas
    dwg.viewbox(x_min - margin_left, y_min - margin_top,
                bb2d_width + 2 * margin_left, bb2d_height + 2 * margin_top)

    for polyline in polylines:
        # apply color and style
        polyline.stroke(color, width=line_width, linecap="round")
        # then adds the polyline to the svg canva
        dwg.add(polyline)

    # export to string or file according to the user choice
    if filename is not None:
        dwg.save()
        if not os.path.isfile(filename):
            raise AssertionError("svg export failed")
        print("Shape successfully exported to %s" % filename)
        return True
    return dwg.tostring()

##############
# X3D export #
##############
X3DFILE_HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE X3D PUBLIC "ISO//Web3D//DTD X3D 4.0//EN" "https://www.web3d.org/specifications/x3d-4.0.dtd">
<X3D profile='Immersive' version='4.0' xmlns:xsd='http://www.w3.org/2001/XMLSchema-instance' xsd:noNamespaceSchemaLocation='http://www.web3d.org/specifications/x3d-4.0.xsd'>
<head>
    <meta name='generator' content='pythonocc-%s X3D exporter (www.pythonocc.org)'/>
    <meta name='creator' content='pythonocc-%s generator'/>
    <meta name='identifier' content='http://www.pythonocc.org'/>
    <meta name='description' content='pythonocc-%s x3dom based shape rendering'/>
</head>
<Scene>
""" % (OCC_VERSION, OCC_VERSION, OCC_VERSION)

X3D_TRIANGLESET_TEMPLATE = """
<TriangleSet solid='false'>
  <Coordinate point='%s'/>
  <Normal vector='%s'/>
</TriangleSet>
"""

X3D_INDEXEDTRIANGLESET_TEMPLATE = """
<IndexedTriangleSet index='%s' solid='false'>
  <Coordinate point='%s'/>
  <Normal vector='%s'/>
</IndexedTriangleSet>
"""


class X3DBaseExporter:
    """ Abstract class that supports common methods for each
    subclass
    """
    def __init__(self,
                 shape,  # the TopoDS shape to mesh
                 vertex_shader="",  # the vertex_shader, passed as a string
                 fragment_shader="",  # the fragment shader, passed as a string
                 export_edges=False,  # if yes, edges are exported to IndexedLineSet (might be SLOWW)
                 color=(0.65, 0.65, 0.7),  # the default shape color
                 specular_color=(0.2, 0.2, 0.2),  # shape specular color (white by default)
                 shininess=0.9,  # shape shininess
                 transparency=0.,  # shape transparency
                 line_color=(0, 0., 0.),  # edge color
                 line_width=2.,  # edge liewidth,
                 mesh_quality=1., # mesh quality default is 1., good is <1, bad is >1
                 verbose=False, # if True, log info related to export,
                 optimize_mesh=True # if true, post process mesh to improve quality/performance
                ):
        self._shape = shape
        self._shape_id = uuid.uuid4().hex
        self._vs = vertex_shader
        self._fs = fragment_shader
        self._export_edges = export_edges
        self._color = color
        self._shininess = shininess
        self._specular_color = specular_color
        self._transparency = transparency
        self._mesh_quality = mesh_quality
        # the list of indexed face sets that compose the shape
        # if ever the map_faces_to_mesh option is enabled, this list
        # maybe composed of dozains of TriangleSet
        self._triangle_sets = []
        self._line_sets = []
        self._x3d_string = ""  # the string that contains the x3d description
        self._computed = False  # will be true when mesh is computed
        self._verbose = False
        # check if geometry can be exported to indexed geometry
        # numpy must be installed
        if HAVE_MESH_TOOLS and optimize_mesh:
            optimize_mesh = True
        else:
            optimize_mesh = False
        self._optimize_mesh = optimize_mesh

    def get_shape_id(self):
        return self._shape_id


class X3DEdgeExporter(X3DBaseExporter):
    """ A class for exporting 1d topology such as TopoDS_Wire or TopoDS_Edge
    This class takes either a TopoDS_Edge, a TopoDS_Wire or a list of TopoDS_Edge
    or a list of TopoDS_Wire.
    In any case, all that is passd to this exporter is exported to a single
    LineSet instance."""
    def __init__(self, *kargs):
        super(X3DEdgeExporter, self).__init__(*kargs)

    def compute(self):
        if is_edge(shape):
            print("X3D exporter, discretize an edge")
            pnts = discretize_edge(shape)
            edge_hash = "edg%s" % uuid.uuid4().hex
            line_set = export_edge_to_lineset(pnts)
            x3dfile_content = lineset_to_x3d_string([line_set], ils_id=edge_hash)
            edge_full_path = os.path.join(self._path, edge_hash + '.x3d')
            with open(edge_full_path, "w") as edge_file:
                edge_file.write(x3dfile_content)
            # store this edge hash
            self._x3d_edges[edge_hash] = [color, line_width]
            return self._x3d_shapes, self._x3d_edges

        if is_wire(shape):
            print("X3D exporter, discretize a wire")
            pnts = discretize_wire(shape)
            wire_hash = "wir%s" % uuid.uuid4().hex
            line_set = export_edge_to_lineset(pnts)
            x3dfile_content = lineset_to_x3d_string([line_set], ils_id=wire_hash)
            wire_full_path = os.path.join(self._path, wire_hash + '.x3d')
            with open(wire_full_path, "w") as wire_file:
                wire_file.write(x3dfile_content)
            # store this edge hash
            self._x3d_edges[wire_hash] = [color, line_width]
            return self._x3d_shapes, self._x3d_edges


class X3DShapeExporter(X3DBaseExporter):
    """ A class for exporting a single TopoDS_Shape to an x3d file """
    def __init__(self, *kargs):
        super(X3DShapeExporter, self).__init__(*kargs)

    def compute(self):
        """ compute meshes, return True if successful
        """
        if self._verbose:
            init_time = time.perf_counter()
            print("start meshing")
        shape_tesselator = ShapeTesselator(self._shape)
        shape_tesselator.Compute(compute_edges=self._export_edges,
                                 mesh_quality=self._mesh_quality,
                                 parallel=True)
        if self._verbose:
            init_time = time.perf_counter()
            print("meshing done")

        points = shape_tesselator.GetVerticesPositionAsTuple()
        normals = shape_tesselator.GetNormalsAsTuple()
        if self._optimize_mesh:
            if self._verbose:
                print("start optimizing mesh")
            idx, vertices, normals = optimize_mesh(shape_tesselator)
            if self._verbose:
                print("converting to indexed geometry done")
            x3d_representation = export_shape_to_X3D_IndexedTriangleSet(idx, u_vertices, normals)
        else:
            x3d_representation = export_shape_to_X3D_TriangleSet(points, normals)
        self._triangle_sets.append(x3d_representation)
        # then process edges
        if self._export_edges:
            all_edges = []  # the list of point coordinates for all edges
            # get number of edges
            nbr_edges = shape_tesselator.ObjGetEdgeCount()
            for i_edge in range(nbr_edges):
                edge_point_set = []
                nbr_vertices = shape_tesselator.ObjEdgeGetVertexCount(i_edge)
                for i_vert in range(nbr_vertices):
                    edge_point_set.append(shape_tesselator.GetEdgeVertex(i_edge, i_vert))
                all_edges.append(edge_point_set)
            #ils = export_edge_to_lineset(edge_point_set)
            ls = export_list_of_edges_to_lineset(all_edges)            
            self._line_sets.append(ls)
        self._computed = True
        if self._verbose:
            final_time = time.perf_counter()
            print("Computation time:", final_time - init_time, "s")
        return True


    def to_x3dfile_string(self):
        x3dfile_str = X3DFILE_HEADER
        for triangle_set in self._triangle_sets:
            x3dfile_str += "<Switch whichChoice='0' id='swBRP'><Transform scale='1 1 1'><Shape DEF='shape%s' onclick='" % self._shape_id
            x3dfile_str += "select(this);"
            x3dfile_str += "'><Appearance>\n"
            #
            # set Material or shader
            #
            if self._vs is None and self._fs is None:
                x3dfile_str += "<Material id='color' diffuseColor="
                x3dfile_str += "'%g %g %g'" % (self._color[0], self._color[1], self._color[2])
                x3dfile_str += " shininess="
                x3dfile_str += "'%g'" % self._shininess
                x3dfile_str += " specularColor="
                x3dfile_str += "'%g %g %g'" % (self._specular_color[0], self._specular_color[1], self._specular_color[2])
                x3dfile_str += " transparency='%g'>\n" % self._transparency
                x3dfile_str += "</Material>\n"
            else:  # set shaders
                x3dfile_str += '<ComposedShader><ShaderPart type="VERTEX" style="display:none;">\n'
                x3dfile_str += self._vs
                x3dfile_str += '</ShaderPart>\n'
                x3dfile_str += '<ShaderPart type="FRAGMENT" style="display:none;">\n'
                x3dfile_str += self._fs
                x3dfile_str += '</ShaderPart></ComposedShader>\n'
            x3dfile_str += '</Appearance>\n'
            # export triangles
            x3dfile_str += triangle_set
            x3dfile_str += "</Shape></Transform></Switch>\n"
        # and now, process edges
        if self._export_edges:
            x3dfile_str += lineset_to_x3d_string(self._line_sets, header=False, footer=False)
        x3dfile_str += '</Scene>\n</X3D>\n'

        return x3dfile_str

    def write_to_file(self, path, filename="", auto_filename=False):
        """ write to a file. If autofilename is set to True then
        the file name is "shp" and the shape id appended.
        """
        if auto_filename:
            filename = "shp%s" % self._shape_id + ".x3d"
        full_filename = os.path.join(path, filename)
        with open(full_filename, "w") as f:
            f.write(self.to_x3dfile_string())
        # check that the file was written
        if not os.path.isfile(full_filename):
            raise IOError("x3d file not written.")
        return filename


def approximate_listoffloat_to_str(list_of_floats, ndigits=3, epsilon=1e-3):
    """ take a list of floats, returns a simplified list
    of formatted floats
    """
    precision_dict = {1: "{0:.1g}", 2: "{0:.2g}", 3: "{0:.3g}", 4: "{0:.4g}", 5: "{0:.5g}",
                      6: "{0:.6g}", 7: "{0:.7g}", 8: "{0:.8g}", 9: "{0:.9g}"}
    listoffloat_to_str = ' '.join(['0' if abs(float_number) < epsilon
                                   else precision_dict[ndigits].format(float_number)
                                   for float_number in list_of_floats])
    return listoffloat_to_str


def export_shape_to_X3D_TriangleSet(points_list,
                                    normals_list,
                                    number_of_digits_for_points_coordinates=4,
                                    number_of_digits_for_normals=2,
                                    epsilon=1e-3):
    """ takes the point_list and normals_list, returns
    a string with the TriangleSet node.
    The points_list and normals_list come from the c++ Tesselator, and
    contain raw float numbers. Their serialization needs a few post process
    in order to reduce precision (thus size) without changing visualization
    that much.
    The following choice is made:
    * for normals, multiply by 100 and only keep the integer part.
    [-0.23931565880775452, 0.9709418416023254, -0.0]
    will become
    (-23 97 0)
    * for point coordinates, format float numbers 
    """
    # format points coordinates using the "g" type,
    # see python doc at
    # https://docs.python.org/3/library/string.html#format-specification-mini-language
    
    points_str = approximate_listoffloat_to_str(points_list)
    normals_str = approximate_listoffloat_to_str(normals_list)

    x3d_triangleset_str = X3D_TRIANGLESET_TEMPLATE % (points_str, normals_str)

    return x3d_triangleset_str


def export_shape_to_X3D_IndexedTriangleSet(idx_lst,
                                           points_list,
                                           normals_list = [],
                                           number_of_digits_for_points_coordinates=4,
                                           epsilon=1e-3):
    index_str = ' '.join(map(str, idx_lst))
    points_str = approximate_listoffloat_to_str(points_list)
    normals_str = approximate_listoffloat_to_str(normals_list)
    x3d_triangleset_str = X3D_INDEXEDTRIANGLESET_TEMPLATE % (index_str,
                                                             points_str,
                                                             normals_str)

    return x3d_triangleset_str


def export_list_of_edges_to_lineset(list_of_edge_point_set):
    """ see issue https://github.com/andreasplesch/OCCToX3D/issues/1
    Yes, you can combine the edges into a single Shape, using LineSet or using IndexedLineSet. Here are examples:
    https://x3dgraphics.com/examples/X3dForWebAuthors/Chapter06GeometryPointsLinesPolygons/LineSetComparisonIndex.html
    Note the vertexCount field of LineSet. It can have multiple entries, each one defining a separate line.
    So the three LineSets in your example would become:
    <LinesSet vertexCount='2 2 2' point='10 20 10, 10 20 40, 10 0 10, 10 20 10, 0 0 10, 10 0 10' />
    
    as input, we get a list of 3 floats list
    """
    # first, we determine the number of points of each edge
    vertexCount_str = ' '.join(["%i" % len(a) for a in list_of_edge_point_set])
    str_x3d_to_return = "\t<LineSet vertexCount='%s'>" % vertexCount_str
    # the we export point coordinates
    # TODO a numpy reshape could here do the job much faster
    coords = []
    for edge_point_set in list_of_edge_point_set:
        for p in edge_point_set:
            coords.append(p[0])
            coords.append(p[1])
            coords.append(p[2])
    # the we build the string from these coords
    coords_str = ' '.join("%g" % c for c in coords)
    str_x3d_to_return += "<Coordinate point='%s'/>\n" % coords_str
    str_x3d_to_return += "</LineSet>\n"
    return str_x3d_to_return


def export_edge_to_lineset(edge_point_set):
    str_x3d_to_return = "\t<LineSet vertexCount='%i'>" % len(edge_point_set)
    str_x3d_to_return += "<Coordinate point='"
    for p in edge_point_set:
        str_x3d_to_return += "%g %g %g " % (p[0], p[1], p[2])
    str_x3d_to_return += "'/></LineSet>\n"
    return str_x3d_to_return


def lineset_to_x3d_string(str_linesets, header=True, footer=True, ils_id=0):
    """ takes an str_lineset, coming for instance from export_curve_to_ils,
    and export to an X3D string"""
    if header:
        x3dfile_str = X3DFILE_HEADER
    else:
        x3dfile_str = ""
    x3dfile_str += "<Switch whichChoice='0' id='swBRP'>\n"
    x3dfile_str += "\t<Group>\n"

    ils_id = 0
    for str_lineset in str_linesets:
        x3dfile_str += "\t\t<Transform scale='1 1 1'><Shape DEF='edg%s'>\n" % ils_id
        # empty appearance, but the x3d validator complains if nothing set
        x3dfile_str += "\t\t\t<Appearance><Material emissiveColor='0 0 0'/></Appearance>\n\t\t"
        x3dfile_str += str_lineset
        x3dfile_str += "\t\t</Shape></Transform>\n"
        ils_id += 1

    x3dfile_str += "\t</Group>\n"
    x3dfile_str += "</Switch>\n"
    if footer:
        x3dfile_str += '</Scene>\n</X3D>\n'

    return x3dfile_str


def write_x3d_file(shape, path, filename):
    x3d_exporter = X3DShapeExporter(shape)
    x3d_exporter.compute()
    x3d_exporter.write_to_file(path, filename)
