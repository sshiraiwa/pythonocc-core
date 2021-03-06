/*
Copyright 2008-2020 Thomas Paviot (tpaviot@gmail.com)

This file is part of pythonOCC.
pythonOCC is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

pythonOCC is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with pythonOCC.  If not, see <http://www.gnu.org/licenses/>.
*/
%define EXPRDOCSTRING
"Expr module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_expr.html"
%enddef
%module (package="OCC.Core", docstring=EXPRDOCSTRING) Expr


%{
#ifdef WNT
#pragma warning(disable : 4716)
#endif
%}

%include ../common/CommonIncludes.i
%include ../common/ExceptionCatcher.i
%include ../common/FunctionTransformers.i
%include ../common/Operators.i
%include ../common/OccHandle.i


%{
#include<Expr_module.hxx>

//Dependencies
#include<Standard_module.hxx>
#include<NCollection_module.hxx>
#include<TColStd_module.hxx>
#include<TCollection_module.hxx>
#include<TColgp_module.hxx>
#include<TColStd_module.hxx>
#include<TCollection_module.hxx>
#include<Storage_module.hxx>
%};
%import Standard.i
%import NCollection.i
%import TColStd.i
%import TCollection.i

%pythoncode {
from enum import IntEnum
from OCC.Core.Exception import *
};

/* public enums */
/* end public enums declaration */

/* python proy classes for enums */
%pythoncode {
};
/* end python proxy for enums */

/* handles */
%wrap_handle(Expr_GeneralExpression)
%wrap_handle(Expr_GeneralFunction)
%wrap_handle(Expr_GeneralRelation)
%wrap_handle(Expr_BinaryExpression)
%wrap_handle(Expr_FunctionDerivative)
%wrap_handle(Expr_NamedExpression)
%wrap_handle(Expr_NamedFunction)
%wrap_handle(Expr_NumericValue)
%wrap_handle(Expr_PolyExpression)
%wrap_handle(Expr_SingleRelation)
%wrap_handle(Expr_SystemRelation)
%wrap_handle(Expr_UnaryExpression)
%wrap_handle(Expr_Absolute)
%wrap_handle(Expr_ArcCosine)
%wrap_handle(Expr_ArcSine)
%wrap_handle(Expr_ArcTangent)
%wrap_handle(Expr_ArgCosh)
%wrap_handle(Expr_ArgSinh)
%wrap_handle(Expr_ArgTanh)
%wrap_handle(Expr_BinaryFunction)
%wrap_handle(Expr_Cosh)
%wrap_handle(Expr_Cosine)
%wrap_handle(Expr_Difference)
%wrap_handle(Expr_Different)
%wrap_handle(Expr_Division)
%wrap_handle(Expr_Equal)
%wrap_handle(Expr_Exponential)
%wrap_handle(Expr_Exponentiate)
%wrap_handle(Expr_GreaterThan)
%wrap_handle(Expr_GreaterThanOrEqual)
%wrap_handle(Expr_LessThan)
%wrap_handle(Expr_LessThanOrEqual)
%wrap_handle(Expr_LogOf10)
%wrap_handle(Expr_LogOfe)
%wrap_handle(Expr_NamedConstant)
%wrap_handle(Expr_NamedUnknown)
%wrap_handle(Expr_PolyFunction)
%wrap_handle(Expr_Product)
%wrap_handle(Expr_Sine)
%wrap_handle(Expr_Sinh)
%wrap_handle(Expr_Square)
%wrap_handle(Expr_SquareRoot)
%wrap_handle(Expr_Sum)
%wrap_handle(Expr_Tangent)
%wrap_handle(Expr_Tanh)
%wrap_handle(Expr_UnaryFunction)
%wrap_handle(Expr_UnaryMinus)
/* end handles declaration */

/* templates */
%template(Expr_Array1OfGeneralExpression) NCollection_Array1<opencascade::handle<Expr_GeneralExpression>>;

%extend NCollection_Array1<opencascade::handle<Expr_GeneralExpression>> {
    %pythoncode {
    def __getitem__(self, index):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            return self.Value(index + self.Lower())

    def __setitem__(self, index, value):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            self.SetValue(index + self.Lower(), value)

    def __len__(self):
        return self.Length()

    def __iter__(self):
        self.low = self.Lower()
        self.up = self.Upper()
        self.current = self.Lower() - 1
        return self

    def next(self):
        if self.current >= self.Upper():
            raise StopIteration
        else:
            self.current += 1
        return self.Value(self.current)

    __next__ = next
    }
};
%template(Expr_Array1OfNamedUnknown) NCollection_Array1<opencascade::handle<Expr_NamedUnknown>>;

%extend NCollection_Array1<opencascade::handle<Expr_NamedUnknown>> {
    %pythoncode {
    def __getitem__(self, index):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            return self.Value(index + self.Lower())

    def __setitem__(self, index, value):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            self.SetValue(index + self.Lower(), value)

    def __len__(self):
        return self.Length()

    def __iter__(self):
        self.low = self.Lower()
        self.up = self.Upper()
        self.current = self.Lower() - 1
        return self

    def next(self):
        if self.current >= self.Upper():
            raise StopIteration
        else:
            self.current += 1
        return self.Value(self.current)

    __next__ = next
    }
};
%template(Expr_Array1OfSingleRelation) NCollection_Array1<opencascade::handle<Expr_SingleRelation>>;

%extend NCollection_Array1<opencascade::handle<Expr_SingleRelation>> {
    %pythoncode {
    def __getitem__(self, index):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            return self.Value(index + self.Lower())

    def __setitem__(self, index, value):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            self.SetValue(index + self.Lower(), value)

    def __len__(self):
        return self.Length()

    def __iter__(self):
        self.low = self.Lower()
        self.up = self.Upper()
        self.current = self.Lower() - 1
        return self

    def next(self):
        if self.current >= self.Upper():
            raise StopIteration
        else:
            self.current += 1
        return self.Value(self.current)

    __next__ = next
    }
};
%template(Expr_MapOfNamedUnknown) NCollection_IndexedMap<opencascade::handle<Expr_NamedUnknown>,TColStd_MapTransientHasher>;
%template(Expr_SequenceOfGeneralExpression) NCollection_Sequence<opencascade::handle<Expr_GeneralExpression>>;
%template(Expr_SequenceOfGeneralRelation) NCollection_Sequence<opencascade::handle<Expr_GeneralRelation>>;
/* end templates declaration */

/* typedefs */
typedef NCollection_Array1<opencascade::handle<Expr_GeneralExpression>> Expr_Array1OfGeneralExpression;
typedef NCollection_Array1<opencascade::handle<Expr_NamedUnknown>> Expr_Array1OfNamedUnknown;
typedef NCollection_Array1<opencascade::handle<Expr_SingleRelation>> Expr_Array1OfSingleRelation;
typedef NCollection_IndexedMap<opencascade::handle<Expr_NamedUnknown>, TColStd_MapTransientHasher> Expr_MapOfNamedUnknown;
typedef NCollection_Sequence<opencascade::handle<Expr_GeneralExpression>> Expr_SequenceOfGeneralExpression;
typedef NCollection_Sequence<opencascade::handle<Expr_GeneralRelation>> Expr_SequenceOfGeneralRelation;
/* end typedefs declaration */

/*************
* class Expr *
*************/
%rename(expr) Expr;
class Expr {
	public:
		/****************** CopyShare ******************/
		%feature("compactdefaultargs") CopyShare;
		%feature("autodoc", "No available documentation.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") CopyShare;
		static opencascade::handle<Expr_GeneralExpression> CopyShare(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** NbOfFreeVariables ******************/
		%feature("compactdefaultargs") NbOfFreeVariables;
		%feature("autodoc", "No available documentation.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
int
") NbOfFreeVariables;
		static Standard_Integer NbOfFreeVariables(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** NbOfFreeVariables ******************/
		%feature("compactdefaultargs") NbOfFreeVariables;
		%feature("autodoc", "No available documentation.

Parameters
----------
exp: Expr_GeneralRelation

Returns
-------
int
") NbOfFreeVariables;
		static Standard_Integer NbOfFreeVariables(const opencascade::handle<Expr_GeneralRelation> & exp);

		/****************** Sign ******************/
		%feature("compactdefaultargs") Sign;
		%feature("autodoc", "No available documentation.

Parameters
----------
val: float

Returns
-------
float
") Sign;
		static Standard_Real Sign(const Standard_Real val);

};


%extend Expr {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*******************************
* class Expr_GeneralExpression *
*******************************/
%nodefaultctor Expr_GeneralExpression;
class Expr_GeneralExpression : public Standard_Transient {
	public:
		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <exp> is contained in <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		virtual Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** ContainsUnknowns ******************/
		%feature("compactdefaultargs") ContainsUnknowns;
		%feature("autodoc", "Tests if <self> contains namedunknowns.

Returns
-------
bool
") ContainsUnknowns;
		virtual Standard_Boolean ContainsUnknowns();

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		virtual opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		virtual opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		virtual Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** EvaluateNumeric ******************/
		%feature("compactdefaultargs") EvaluateNumeric;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Returns
-------
float
") EvaluateNumeric;
		Standard_Real EvaluateNumeric();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. warning: this method does not include any simplification before testing. it could also be very slow; to be used carefully.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		virtual Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "Tests if <self> is linear on every namedunknown it contains.

Returns
-------
bool
") IsLinear;
		virtual Standard_Boolean IsLinear();

		/****************** IsShareable ******************/
		%feature("compactdefaultargs") IsShareable;
		%feature("autodoc", "Tests if <self> can be shared by one or more expressions or must be copied. this method returns false as a default value. to be redefined ( especially for namedunknown).

Returns
-------
bool
") IsShareable;
		virtual Standard_Boolean IsShareable();

		/****************** NDerivative ******************/
		%feature("compactdefaultargs") NDerivative;
		%feature("autodoc", "Returns the <n>-th derivative on <x> unknown of <self>. raise outofrange if n <= 0.

Parameters
----------
X: Expr_NamedUnknown
N: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") NDerivative;
		virtual opencascade::handle<Expr_GeneralExpression> NDerivative(const opencascade::handle<Expr_NamedUnknown> & X, const Standard_Integer N);

		/****************** NbSubExpressions ******************/
		%feature("compactdefaultargs") NbSubExpressions;
		%feature("autodoc", "Returns the number of sub-expressions contained in <self> ( >= 0).

Returns
-------
int
") NbSubExpressions;
		virtual Standard_Integer NbSubExpressions();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with copies of <with> in <self>. copies of <with> are made with the copy() method. raises invalidoperand if <with> contains <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		virtual void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		virtual opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalexpression after replacement of namedunknowns by an associated expression and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Simplified;
		virtual opencascade::handle<Expr_GeneralExpression> Simplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		virtual TCollection_AsciiString String();

		/****************** SubExpression ******************/
		%feature("compactdefaultargs") SubExpression;
		%feature("autodoc", "Returns the <i>-th sub-expression of <self> raises outofrange if <i> > nbsubexpressions(me).

Parameters
----------
I: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SubExpression;
		virtual const opencascade::handle<Expr_GeneralExpression> & SubExpression(const Standard_Integer I);

};


%make_alias(Expr_GeneralExpression)

%extend Expr_GeneralExpression {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*****************************
* class Expr_GeneralFunction *
*****************************/
%nodefaultctor Expr_GeneralFunction;
class Expr_GeneralFunction : public Standard_Transient {
	public:
		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> with the same form.

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Copy;
		virtual opencascade::handle<Expr_GeneralFunction> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns derivative of <self> for variable <var>.

Parameters
----------
var: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Derivative;
		virtual opencascade::handle<Expr_GeneralFunction> Derivative(const opencascade::handle<Expr_NamedUnknown> & var);

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns derivative of <self> for variable <var> with degree <deg>.

Parameters
----------
var: Expr_NamedUnknown
deg: int

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Derivative;
		virtual opencascade::handle<Expr_GeneralFunction> Derivative(const opencascade::handle<Expr_NamedUnknown> & var, const Standard_Integer deg);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Computes the value of <self> with the given variables. raises notevaluable if <vars> does not match all variables of <self>.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		virtual Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** GetStringName ******************/
		%feature("compactdefaultargs") GetStringName;
		%feature("autodoc", "No available documentation.

Returns
-------
TCollection_AsciiString
") GetStringName;
		virtual TCollection_AsciiString GetStringName();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <func> are similar functions (same name and same used expression).

Parameters
----------
func: Expr_GeneralFunction

Returns
-------
bool
") IsIdentical;
		virtual Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralFunction> & func);

		/****************** IsLinearOnVariable ******************/
		%feature("compactdefaultargs") IsLinearOnVariable;
		%feature("autodoc", "Tests if <self> is linear on variable on range <index>.

Parameters
----------
index: int

Returns
-------
bool
") IsLinearOnVariable;
		virtual Standard_Boolean IsLinearOnVariable(const Standard_Integer index);

		/****************** NbOfVariables ******************/
		%feature("compactdefaultargs") NbOfVariables;
		%feature("autodoc", "Returns the number of variables of <self>.

Returns
-------
int
") NbOfVariables;
		virtual Standard_Integer NbOfVariables();

		/****************** Variable ******************/
		%feature("compactdefaultargs") Variable;
		%feature("autodoc", "Returns the variable denoted by <index> in <self>. raises outofrange if index > nbofvariables.

Parameters
----------
index: int

Returns
-------
opencascade::handle<Expr_NamedUnknown>
") Variable;
		virtual opencascade::handle<Expr_NamedUnknown> Variable(const Standard_Integer index);

};


%make_alias(Expr_GeneralFunction)

%extend Expr_GeneralFunction {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*****************************
* class Expr_GeneralRelation *
*****************************/
%nodefaultctor Expr_GeneralRelation;
class Expr_GeneralRelation : public Standard_Transient {
	public:
		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <exp> contains <var>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		virtual Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		virtual opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "Tests if <self> is linear between its namedunknowns.

Returns
-------
bool
") IsLinear;
		virtual Standard_Boolean IsLinear();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "Returns the current status of the relation.

Returns
-------
bool
") IsSatisfied;
		virtual Standard_Boolean IsSatisfied();

		/****************** NbOfSingleRelations ******************/
		%feature("compactdefaultargs") NbOfSingleRelations;
		%feature("autodoc", "Returns the number of singlerelations contained in <self>.

Returns
-------
int
") NbOfSingleRelations;
		virtual Standard_Integer NbOfSingleRelations();

		/****************** NbOfSubRelations ******************/
		%feature("compactdefaultargs") NbOfSubRelations;
		%feature("autodoc", "Returns the number of relations contained in <self>.

Returns
-------
int
") NbOfSubRelations;
		virtual Standard_Integer NbOfSubRelations();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		virtual void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		virtual opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by associated expressions, and computes values in <self>.

Returns
-------
None
") Simplify;
		virtual void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		virtual TCollection_AsciiString String();

		/****************** SubRelation ******************/
		%feature("compactdefaultargs") SubRelation;
		%feature("autodoc", "Returns the relation denoted by <index> in <self>. an exception is raised if <index> is out of range.

Parameters
----------
index: int

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") SubRelation;
		virtual opencascade::handle<Expr_GeneralRelation> SubRelation(const Standard_Integer index);

};


%make_alias(Expr_GeneralRelation)

%extend Expr_GeneralRelation {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/************************
* class Expr_RUIterator *
************************/
class Expr_RUIterator {
	public:
		/****************** Expr_RUIterator ******************/
		%feature("compactdefaultargs") Expr_RUIterator;
		%feature("autodoc", "Creates an iterator on every namedunknown contained in <rel>.

Parameters
----------
rel: Expr_GeneralRelation

Returns
-------
None
") Expr_RUIterator;
		 Expr_RUIterator(const opencascade::handle<Expr_GeneralRelation> & rel);

		/****************** More ******************/
		%feature("compactdefaultargs") More;
		%feature("autodoc", "Returns false if on other unknown remains.

Returns
-------
bool
") More;
		Standard_Boolean More();

		/****************** Next ******************/
		%feature("compactdefaultargs") Next;
		%feature("autodoc", "No available documentation.

Returns
-------
None
") Next;
		void Next();

		/****************** Value ******************/
		%feature("compactdefaultargs") Value;
		%feature("autodoc", "Returns current namedunknown. raises exception if no more unknowns remain.

Returns
-------
opencascade::handle<Expr_NamedUnknown>
") Value;
		opencascade::handle<Expr_NamedUnknown> Value();

};


%extend Expr_RUIterator {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/******************************
* class Expr_RelationIterator *
******************************/
class Expr_RelationIterator {
	public:
		/****************** Expr_RelationIterator ******************/
		%feature("compactdefaultargs") Expr_RelationIterator;
		%feature("autodoc", "No available documentation.

Parameters
----------
rel: Expr_GeneralRelation

Returns
-------
None
") Expr_RelationIterator;
		 Expr_RelationIterator(const opencascade::handle<Expr_GeneralRelation> & rel);

		/****************** More ******************/
		%feature("compactdefaultargs") More;
		%feature("autodoc", "Returns false if no other relation remains.

Returns
-------
bool
") More;
		Standard_Boolean More();

		/****************** Next ******************/
		%feature("compactdefaultargs") Next;
		%feature("autodoc", "No available documentation.

Returns
-------
None
") Next;
		void Next();

		/****************** Value ******************/
		%feature("compactdefaultargs") Value;
		%feature("autodoc", "Returns current basic relation. exception is raised if no more relation remains.

Returns
-------
opencascade::handle<Expr_SingleRelation>
") Value;
		opencascade::handle<Expr_SingleRelation> Value();

};


%extend Expr_RelationIterator {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*****************************
* class Expr_UnknownIterator *
*****************************/
class Expr_UnknownIterator {
	public:
		/****************** Expr_UnknownIterator ******************/
		%feature("compactdefaultargs") Expr_UnknownIterator;
		%feature("autodoc", "No available documentation.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_UnknownIterator;
		 Expr_UnknownIterator(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** More ******************/
		%feature("compactdefaultargs") More;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") More;
		Standard_Boolean More();

		/****************** Next ******************/
		%feature("compactdefaultargs") Next;
		%feature("autodoc", "No available documentation.

Returns
-------
None
") Next;
		void Next();

		/****************** Value ******************/
		%feature("compactdefaultargs") Value;
		%feature("autodoc", "No available documentation.

Returns
-------
opencascade::handle<Expr_NamedUnknown>
") Value;
		opencascade::handle<Expr_NamedUnknown> Value();

};


%extend Expr_UnknownIterator {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/******************************
* class Expr_BinaryExpression *
******************************/
%nodefaultctor Expr_BinaryExpression;
class Expr_BinaryExpression : public Expr_GeneralExpression {
	public:
		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <self> contains <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** ContainsUnknowns ******************/
		%feature("compactdefaultargs") ContainsUnknowns;
		%feature("autodoc", "Does <self> contain namedunknown ?.

Returns
-------
bool
") ContainsUnknowns;
		Standard_Boolean ContainsUnknowns();

		/****************** FirstOperand ******************/
		%feature("compactdefaultargs") FirstOperand;
		%feature("autodoc", "No available documentation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") FirstOperand;
		const opencascade::handle<Expr_GeneralExpression> & FirstOperand();

		/****************** NbSubExpressions ******************/
		%feature("compactdefaultargs") NbSubExpressions;
		%feature("autodoc", "Returns the number of sub-expressions contained in <self> ( >= 0).

Returns
-------
int
") NbSubExpressions;
		Standard_Integer NbSubExpressions();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self>. raises invalidoperand if <with> contains <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** SecondOperand ******************/
		%feature("compactdefaultargs") SecondOperand;
		%feature("autodoc", "No available documentation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SecondOperand;
		const opencascade::handle<Expr_GeneralExpression> & SecondOperand();

		/****************** SetFirstOperand ******************/
		%feature("compactdefaultargs") SetFirstOperand;
		%feature("autodoc", "Sets first operand of <self> raises invalidoperand if exp = me.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") SetFirstOperand;
		void SetFirstOperand(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** SetSecondOperand ******************/
		%feature("compactdefaultargs") SetSecondOperand;
		%feature("autodoc", "Sets second operand of <self> raises invalidoperand if <exp> contains <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") SetSecondOperand;
		void SetSecondOperand(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalexpression after replacement of namedunknowns by an associated expression and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Simplified;
		opencascade::handle<Expr_GeneralExpression> Simplified();

		/****************** SubExpression ******************/
		%feature("compactdefaultargs") SubExpression;
		%feature("autodoc", "Returns the <i>-th sub-expression of <self> raises outofrange if <i> > nbsubexpressions(me).

Parameters
----------
I: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SubExpression;
		const opencascade::handle<Expr_GeneralExpression> & SubExpression(const Standard_Integer I);

};


%make_alias(Expr_BinaryExpression)

%extend Expr_BinaryExpression {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/********************************
* class Expr_FunctionDerivative *
********************************/
class Expr_FunctionDerivative : public Expr_GeneralFunction {
	public:
		/****************** Expr_FunctionDerivative ******************/
		%feature("compactdefaultargs") Expr_FunctionDerivative;
		%feature("autodoc", "Creates a functionderivative of degree <deg> relative to the <withx> variable. raises outofrange if <deg> lower or equal to zero.

Parameters
----------
func: Expr_GeneralFunction
withX: Expr_NamedUnknown
deg: int

Returns
-------
None
") Expr_FunctionDerivative;
		 Expr_FunctionDerivative(const opencascade::handle<Expr_GeneralFunction> & func, const opencascade::handle<Expr_NamedUnknown> & withX, const Standard_Integer deg);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> with the same form.

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Copy;
		opencascade::handle<Expr_GeneralFunction> Copy();

		/****************** Degree ******************/
		%feature("compactdefaultargs") Degree;
		%feature("autodoc", "Returns the degree of derivation of <self>.

Returns
-------
int
") Degree;
		Standard_Integer Degree();

		/****************** DerivVariable ******************/
		%feature("compactdefaultargs") DerivVariable;
		%feature("autodoc", "Returns the derivation variable of <self>.

Returns
-------
opencascade::handle<Expr_NamedUnknown>
") DerivVariable;
		opencascade::handle<Expr_NamedUnknown> DerivVariable();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns derivative of <self> for variable <var>.

Parameters
----------
var: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Derivative;
		opencascade::handle<Expr_GeneralFunction> Derivative(const opencascade::handle<Expr_NamedUnknown> & var);

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns derivative of <self> for variable <var> with degree <deg>.

Parameters
----------
var: Expr_NamedUnknown
deg: int

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Derivative;
		opencascade::handle<Expr_GeneralFunction> Derivative(const opencascade::handle<Expr_NamedUnknown> & var, const Standard_Integer deg);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Computes the value of <self> with the given variables. raises dimensionmismatch if length(vars) is different from length(values).

Parameters
----------
vars: Expr_Array1OfNamedUnknown
values: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & values);

		/****************** Expression ******************/
		%feature("compactdefaultargs") Expression;
		%feature("autodoc", "No available documentation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Expression;
		opencascade::handle<Expr_GeneralExpression> Expression();

		/****************** Function ******************/
		%feature("compactdefaultargs") Function;
		%feature("autodoc", "Returns the function of which <self> is the derivative.

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Function;
		opencascade::handle<Expr_GeneralFunction> Function();

		/****************** GetStringName ******************/
		%feature("compactdefaultargs") GetStringName;
		%feature("autodoc", "No available documentation.

Returns
-------
TCollection_AsciiString
") GetStringName;
		TCollection_AsciiString GetStringName();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <func> are similar functions (same name and same used expression).

Parameters
----------
func: Expr_GeneralFunction

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralFunction> & func);

		/****************** IsLinearOnVariable ******************/
		%feature("compactdefaultargs") IsLinearOnVariable;
		%feature("autodoc", "Tests if <self> is linear on variable on range <index>.

Parameters
----------
index: int

Returns
-------
bool
") IsLinearOnVariable;
		Standard_Boolean IsLinearOnVariable(const Standard_Integer index);

		/****************** NbOfVariables ******************/
		%feature("compactdefaultargs") NbOfVariables;
		%feature("autodoc", "Returns the number of variables of <self>.

Returns
-------
int
") NbOfVariables;
		Standard_Integer NbOfVariables();

		/****************** UpdateExpression ******************/
		%feature("compactdefaultargs") UpdateExpression;
		%feature("autodoc", "No available documentation.

Returns
-------
None
") UpdateExpression;
		void UpdateExpression();

		/****************** Variable ******************/
		%feature("compactdefaultargs") Variable;
		%feature("autodoc", "Returns the variable denoted by <index> in <self>. raises outofrange if <index> greater than nbofvariables of <self>.

Parameters
----------
index: int

Returns
-------
opencascade::handle<Expr_NamedUnknown>
") Variable;
		opencascade::handle<Expr_NamedUnknown> Variable(const Standard_Integer index);

};


%make_alias(Expr_FunctionDerivative)

%extend Expr_FunctionDerivative {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*****************************
* class Expr_NamedExpression *
*****************************/
%nodefaultctor Expr_NamedExpression;
class Expr_NamedExpression : public Expr_GeneralExpression {
	public:
		/****************** GetName ******************/
		%feature("compactdefaultargs") GetName;
		%feature("autodoc", "No available documentation.

Returns
-------
TCollection_AsciiString
") GetName;
		const TCollection_AsciiString & GetName();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsShareable ******************/
		%feature("compactdefaultargs") IsShareable;
		%feature("autodoc", "Tests if <self> can be shared by one or more expressions or must be copied. this method redefines to a true value the generalexpression method.

Returns
-------
bool
") IsShareable;
		virtual Standard_Boolean IsShareable();

		/****************** SetName ******************/
		%feature("compactdefaultargs") SetName;
		%feature("autodoc", "No available documentation.

Parameters
----------
name: TCollection_AsciiString

Returns
-------
None
") SetName;
		void SetName(const TCollection_AsciiString & name);

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_NamedExpression)

%extend Expr_NamedExpression {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/***************************
* class Expr_NamedFunction *
***************************/
class Expr_NamedFunction : public Expr_GeneralFunction {
	public:
		/****************** Expr_NamedFunction ******************/
		%feature("compactdefaultargs") Expr_NamedFunction;
		%feature("autodoc", "Creates a function of given variables <vars> with name <name> defined by the expression <exp>.

Parameters
----------
name: TCollection_AsciiString
exp: Expr_GeneralExpression
vars: Expr_Array1OfNamedUnknown

Returns
-------
None
") Expr_NamedFunction;
		 Expr_NamedFunction(const TCollection_AsciiString & name, const opencascade::handle<Expr_GeneralExpression> & exp, const Expr_Array1OfNamedUnknown & vars);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> with the same form.

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Copy;
		opencascade::handle<Expr_GeneralFunction> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns derivative of <self> for variable <var>.

Parameters
----------
var: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Derivative;
		opencascade::handle<Expr_GeneralFunction> Derivative(const opencascade::handle<Expr_NamedUnknown> & var);

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns derivative of <self> for variable <var> with degree <deg>.

Parameters
----------
var: Expr_NamedUnknown
deg: int

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Derivative;
		opencascade::handle<Expr_GeneralFunction> Derivative(const opencascade::handle<Expr_NamedUnknown> & var, const Standard_Integer deg);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Computes the value of <self> with the given variables. raises dimensionmismatch if length(vars) is different from length(values).

Parameters
----------
vars: Expr_Array1OfNamedUnknown
values: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & values);

		/****************** Expression ******************/
		%feature("compactdefaultargs") Expression;
		%feature("autodoc", "Returns equivalent expression of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Expression;
		opencascade::handle<Expr_GeneralExpression> Expression();

		/****************** GetName ******************/
		%feature("compactdefaultargs") GetName;
		%feature("autodoc", "Returns the name assigned to <self>.

Returns
-------
TCollection_AsciiString
") GetName;
		TCollection_AsciiString GetName();

		/****************** GetStringName ******************/
		%feature("compactdefaultargs") GetStringName;
		%feature("autodoc", "No available documentation.

Returns
-------
TCollection_AsciiString
") GetStringName;
		TCollection_AsciiString GetStringName();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <func> are similar functions (same name and same used expression).

Parameters
----------
func: Expr_GeneralFunction

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralFunction> & func);

		/****************** IsLinearOnVariable ******************/
		%feature("compactdefaultargs") IsLinearOnVariable;
		%feature("autodoc", "Tests if <self> is linear on variable on range <index>.

Parameters
----------
index: int

Returns
-------
bool
") IsLinearOnVariable;
		Standard_Boolean IsLinearOnVariable(const Standard_Integer index);

		/****************** NbOfVariables ******************/
		%feature("compactdefaultargs") NbOfVariables;
		%feature("autodoc", "Returns the number of variables of <self>.

Returns
-------
int
") NbOfVariables;
		Standard_Integer NbOfVariables();

		/****************** SetExpression ******************/
		%feature("compactdefaultargs") SetExpression;
		%feature("autodoc", "Modifies expression of <self>. warning: beware of derivatives. see functionderivative.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") SetExpression;
		void SetExpression(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** SetName ******************/
		%feature("compactdefaultargs") SetName;
		%feature("autodoc", "Sets the name <newname> to <self>.

Parameters
----------
newname: TCollection_AsciiString

Returns
-------
None
") SetName;
		void SetName(const TCollection_AsciiString & newname);

		/****************** Variable ******************/
		%feature("compactdefaultargs") Variable;
		%feature("autodoc", "Returns the variable denoted by <index> in <self>. raises outofrange if <index> is greater than nbofvariables of <self>, or less than or equal to zero.

Parameters
----------
index: int

Returns
-------
opencascade::handle<Expr_NamedUnknown>
") Variable;
		opencascade::handle<Expr_NamedUnknown> Variable(const Standard_Integer index);

};


%make_alias(Expr_NamedFunction)

%extend Expr_NamedFunction {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/**************************
* class Expr_NumericValue *
**************************/
class Expr_NumericValue : public Expr_GeneralExpression {
	public:
		/****************** Expr_NumericValue ******************/
		%feature("compactdefaultargs") Expr_NumericValue;
		%feature("autodoc", "No available documentation.

Parameters
----------
val: float

Returns
-------
None
") Expr_NumericValue;
		 Expr_NumericValue(const Standard_Real val);

		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <exp> is contained in <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** ContainsUnknowns ******************/
		%feature("compactdefaultargs") ContainsUnknowns;
		%feature("autodoc", "Tests if <self> contains namedunknown.

Returns
-------
bool
") ContainsUnknowns;
		Standard_Boolean ContainsUnknowns();

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** GetValue ******************/
		%feature("compactdefaultargs") GetValue;
		%feature("autodoc", "No available documentation.

Returns
-------
float
") GetValue;
		Standard_Real GetValue();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** NDerivative ******************/
		%feature("compactdefaultargs") NDerivative;
		%feature("autodoc", "Returns the <n>-th derivative on <x> unknown of <self>. raises outofrange if <n> <= 0.

Parameters
----------
X: Expr_NamedUnknown
N: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") NDerivative;
		virtual opencascade::handle<Expr_GeneralExpression> NDerivative(const opencascade::handle<Expr_NamedUnknown> & X, const Standard_Integer N);

		/****************** NbSubExpressions ******************/
		%feature("compactdefaultargs") NbSubExpressions;
		%feature("autodoc", "Returns the number of sub-expressions contained in <self> ( >= 0).

Returns
-------
int
") NbSubExpressions;
		Standard_Integer NbSubExpressions();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** SetValue ******************/
		%feature("compactdefaultargs") SetValue;
		%feature("autodoc", "No available documentation.

Parameters
----------
val: float

Returns
-------
None
") SetValue;
		void SetValue(const Standard_Real val);

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalexpression after replacement of namedunknowns by an associated expression and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Simplified;
		opencascade::handle<Expr_GeneralExpression> Simplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

		/****************** SubExpression ******************/
		%feature("compactdefaultargs") SubExpression;
		%feature("autodoc", "Returns the <i>-th sub-expression of <self> raises outofrange if <i> > nbsubexpressions(me).

Parameters
----------
I: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SubExpression;
		const opencascade::handle<Expr_GeneralExpression> & SubExpression(const Standard_Integer I);

};


%make_alias(Expr_NumericValue)

%extend Expr_NumericValue {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/****************************
* class Expr_PolyExpression *
****************************/
%nodefaultctor Expr_PolyExpression;
class Expr_PolyExpression : public Expr_GeneralExpression {
	public:
		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <exp> is contained in <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** ContainsUnknowns ******************/
		%feature("compactdefaultargs") ContainsUnknowns;
		%feature("autodoc", "Does <self> contains namedunknown ?.

Returns
-------
bool
") ContainsUnknowns;
		Standard_Boolean ContainsUnknowns();

		/****************** NbOperands ******************/
		%feature("compactdefaultargs") NbOperands;
		%feature("autodoc", "Returns the number of operands contained in <self>.

Returns
-------
int
") NbOperands;
		Standard_Integer NbOperands();

		/****************** NbSubExpressions ******************/
		%feature("compactdefaultargs") NbSubExpressions;
		%feature("autodoc", "Returns the number of sub-expressions contained in <self> ( >= 2).

Returns
-------
int
") NbSubExpressions;
		Standard_Integer NbSubExpressions();

		/****************** Operand ******************/
		%feature("compactdefaultargs") Operand;
		%feature("autodoc", "Returns the <index>-th operand used in <self>. an exception is raised if index is out of range.

Parameters
----------
index: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Operand;
		const opencascade::handle<Expr_GeneralExpression> & Operand(const Standard_Integer index);

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self> raises invalidoperand if <with> contains <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** SetOperand ******************/
		%feature("compactdefaultargs") SetOperand;
		%feature("autodoc", "Sets the <index>-th operand used in <self>. an exception is raised if <index> is out of range raises invalidoperand if <exp> contains <self>.

Parameters
----------
exp: Expr_GeneralExpression
index: int

Returns
-------
None
") SetOperand;
		void SetOperand(const opencascade::handle<Expr_GeneralExpression> & exp, const Standard_Integer index);

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalexpression after replacement of namedunknowns by an associated expression and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Simplified;
		opencascade::handle<Expr_GeneralExpression> Simplified();

		/****************** SubExpression ******************/
		%feature("compactdefaultargs") SubExpression;
		%feature("autodoc", "Returns the sub-expression denoted by <i> in <self> raises outofrange if <i> > nbsubexpressions(me).

Parameters
----------
I: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SubExpression;
		const opencascade::handle<Expr_GeneralExpression> & SubExpression(const Standard_Integer I);

};


%make_alias(Expr_PolyExpression)

%extend Expr_PolyExpression {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/****************************
* class Expr_SingleRelation *
****************************/
%nodefaultctor Expr_SingleRelation;
class Expr_SingleRelation : public Expr_GeneralRelation {
	public:
		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <self> contains <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** FirstMember ******************/
		%feature("compactdefaultargs") FirstMember;
		%feature("autodoc", "Returns the first member of the relation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") FirstMember;
		opencascade::handle<Expr_GeneralExpression> FirstMember();

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "Tests if <self> is linear between its namedunknowns.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** NbOfSingleRelations ******************/
		%feature("compactdefaultargs") NbOfSingleRelations;
		%feature("autodoc", "Returns the number of singlerelations contained in <self> (always 1).

Returns
-------
int
") NbOfSingleRelations;
		Standard_Integer NbOfSingleRelations();

		/****************** NbOfSubRelations ******************/
		%feature("compactdefaultargs") NbOfSubRelations;
		%feature("autodoc", "Returns the number of relations contained in <self>.

Returns
-------
int
") NbOfSubRelations;
		Standard_Integer NbOfSubRelations();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** SecondMember ******************/
		%feature("compactdefaultargs") SecondMember;
		%feature("autodoc", "Returns the second member of the relation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SecondMember;
		opencascade::handle<Expr_GeneralExpression> SecondMember();

		/****************** SetFirstMember ******************/
		%feature("compactdefaultargs") SetFirstMember;
		%feature("autodoc", "Defines the first member of the relation.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") SetFirstMember;
		void SetFirstMember(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** SetSecondMember ******************/
		%feature("compactdefaultargs") SetSecondMember;
		%feature("autodoc", "Defines the second member of the relation.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") SetSecondMember;
		void SetSecondMember(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** SubRelation ******************/
		%feature("compactdefaultargs") SubRelation;
		%feature("autodoc", "Returns the relation denoted by <index> in <self>. an exception is raised if index is out of range.

Parameters
----------
index: int

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") SubRelation;
		opencascade::handle<Expr_GeneralRelation> SubRelation(const Standard_Integer index);

};


%make_alias(Expr_SingleRelation)

%extend Expr_SingleRelation {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/****************************
* class Expr_SystemRelation *
****************************/
class Expr_SystemRelation : public Expr_GeneralRelation {
	public:
		/****************** Expr_SystemRelation ******************/
		%feature("compactdefaultargs") Expr_SystemRelation;
		%feature("autodoc", "Creates a system with one relation.

Parameters
----------
relation: Expr_GeneralRelation

Returns
-------
None
") Expr_SystemRelation;
		 Expr_SystemRelation(const opencascade::handle<Expr_GeneralRelation> & relation);

		/****************** Add ******************/
		%feature("compactdefaultargs") Add;
		%feature("autodoc", "Appends <relation> in the list of components of <self>.

Parameters
----------
relation: Expr_GeneralRelation

Returns
-------
None
") Add;
		void Add(const opencascade::handle<Expr_GeneralRelation> & relation);

		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <self> contains <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "Tests if <self> is linear between its namedunknowns.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsSatisfied;
		Standard_Boolean IsSatisfied();

		/****************** NbOfSingleRelations ******************/
		%feature("compactdefaultargs") NbOfSingleRelations;
		%feature("autodoc", "Returns the number of singlerelations contained in <self>.

Returns
-------
int
") NbOfSingleRelations;
		Standard_Integer NbOfSingleRelations();

		/****************** NbOfSubRelations ******************/
		%feature("compactdefaultargs") NbOfSubRelations;
		%feature("autodoc", "Returns the number of relations contained in <self>.

Returns
-------
int
") NbOfSubRelations;
		Standard_Integer NbOfSubRelations();

		/****************** Remove ******************/
		%feature("compactdefaultargs") Remove;
		%feature("autodoc", "No available documentation.

Parameters
----------
relation: Expr_GeneralRelation

Returns
-------
None
") Remove;
		void Remove(const opencascade::handle<Expr_GeneralRelation> & relation);

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by associated expressions, and computes values in <self>.

Returns
-------
None
") Simplify;
		void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

		/****************** SubRelation ******************/
		%feature("compactdefaultargs") SubRelation;
		%feature("autodoc", "Returns the relation denoted by <index> in <self>. an exception is raised if <index> is out of range.

Parameters
----------
index: int

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") SubRelation;
		opencascade::handle<Expr_GeneralRelation> SubRelation(const Standard_Integer index);

};


%make_alias(Expr_SystemRelation)

%extend Expr_SystemRelation {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*****************************
* class Expr_UnaryExpression *
*****************************/
%nodefaultctor Expr_UnaryExpression;
class Expr_UnaryExpression : public Expr_GeneralExpression {
	public:
		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <exp> is contained in <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** ContainsUnknowns ******************/
		%feature("compactdefaultargs") ContainsUnknowns;
		%feature("autodoc", "Does <self> contains namedunknown ?.

Returns
-------
bool
") ContainsUnknowns;
		Standard_Boolean ContainsUnknowns();

		/****************** NbSubExpressions ******************/
		%feature("compactdefaultargs") NbSubExpressions;
		%feature("autodoc", "Returns the number of sub-expressions contained in <self> ( >= 0).

Returns
-------
int
") NbSubExpressions;
		Standard_Integer NbSubExpressions();

		/****************** Operand ******************/
		%feature("compactdefaultargs") Operand;
		%feature("autodoc", "Returns the operand used.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Operand;
		const opencascade::handle<Expr_GeneralExpression> & Operand();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self> raises invalidoperand if <with> contains <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** SetOperand ******************/
		%feature("compactdefaultargs") SetOperand;
		%feature("autodoc", "Sets the operand used raises invalidoperand if <exp> contains <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") SetOperand;
		void SetOperand(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalexpression after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Simplified;
		opencascade::handle<Expr_GeneralExpression> Simplified();

		/****************** SubExpression ******************/
		%feature("compactdefaultargs") SubExpression;
		%feature("autodoc", "Returns the <i>-th sub-expression of <self>. raises outofrange if <i> > nbsubexpressions(me).

Parameters
----------
I: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SubExpression;
		const opencascade::handle<Expr_GeneralExpression> & SubExpression(const Standard_Integer I);

};


%make_alias(Expr_UnaryExpression)

%extend Expr_UnaryExpression {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/**********************
* class Expr_Absolute *
**********************/
class Expr_Absolute : public Expr_UnaryExpression {
	public:
		/****************** Expr_Absolute ******************/
		%feature("compactdefaultargs") Expr_Absolute;
		%feature("autodoc", "Creates the abs of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Absolute;
		 Expr_Absolute(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Absolute)

%extend Expr_Absolute {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/***********************
* class Expr_ArcCosine *
***********************/
class Expr_ArcCosine : public Expr_UnaryExpression {
	public:
		/****************** Expr_ArcCosine ******************/
		%feature("compactdefaultargs") Expr_ArcCosine;
		%feature("autodoc", "Creates the arccos of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_ArcCosine;
		 Expr_ArcCosine(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_ArcCosine)

%extend Expr_ArcCosine {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*********************
* class Expr_ArcSine *
*********************/
class Expr_ArcSine : public Expr_UnaryExpression {
	public:
		/****************** Expr_ArcSine ******************/
		%feature("compactdefaultargs") Expr_ArcSine;
		%feature("autodoc", "Creates the arcsin of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_ArcSine;
		 Expr_ArcSine(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_ArcSine)

%extend Expr_ArcSine {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/************************
* class Expr_ArcTangent *
************************/
class Expr_ArcTangent : public Expr_UnaryExpression {
	public:
		/****************** Expr_ArcTangent ******************/
		%feature("compactdefaultargs") Expr_ArcTangent;
		%feature("autodoc", "Creates the arctan of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_ArcTangent;
		 Expr_ArcTangent(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_ArcTangent)

%extend Expr_ArcTangent {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*********************
* class Expr_ArgCosh *
*********************/
class Expr_ArgCosh : public Expr_UnaryExpression {
	public:
		/****************** Expr_ArgCosh ******************/
		%feature("compactdefaultargs") Expr_ArgCosh;
		%feature("autodoc", "Creates the argcosh of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_ArgCosh;
		 Expr_ArgCosh(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_ArgCosh)

%extend Expr_ArgCosh {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*********************
* class Expr_ArgSinh *
*********************/
class Expr_ArgSinh : public Expr_UnaryExpression {
	public:
		/****************** Expr_ArgSinh ******************/
		%feature("compactdefaultargs") Expr_ArgSinh;
		%feature("autodoc", "Creates the argsinh of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_ArgSinh;
		 Expr_ArgSinh(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_ArgSinh)

%extend Expr_ArgSinh {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*********************
* class Expr_ArgTanh *
*********************/
class Expr_ArgTanh : public Expr_UnaryExpression {
	public:
		/****************** Expr_ArgTanh ******************/
		%feature("compactdefaultargs") Expr_ArgTanh;
		%feature("autodoc", "Creates the argtanh of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_ArgTanh;
		 Expr_ArgTanh(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_ArgTanh)

%extend Expr_ArgTanh {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/****************************
* class Expr_BinaryFunction *
****************************/
class Expr_BinaryFunction : public Expr_BinaryExpression {
	public:
		/****************** Expr_BinaryFunction ******************/
		%feature("compactdefaultargs") Expr_BinaryFunction;
		%feature("autodoc", "Creates <self> as <func> (<exp1>,<exp2>). raises exception if <func> is not binary.

Parameters
----------
func: Expr_GeneralFunction
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_BinaryFunction;
		 Expr_BinaryFunction(const opencascade::handle<Expr_GeneralFunction> & func, const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** Function ******************/
		%feature("compactdefaultargs") Function;
		%feature("autodoc", "Returns the function defining <self>.

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Function;
		opencascade::handle<Expr_GeneralFunction> Function();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_BinaryFunction)

%extend Expr_BinaryFunction {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/******************
* class Expr_Cosh *
******************/
class Expr_Cosh : public Expr_UnaryExpression {
	public:
		/****************** Expr_Cosh ******************/
		%feature("compactdefaultargs") Expr_Cosh;
		%feature("autodoc", "Creates the cosh of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Cosh;
		 Expr_Cosh(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Cosh)

%extend Expr_Cosh {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/********************
* class Expr_Cosine *
********************/
class Expr_Cosine : public Expr_UnaryExpression {
	public:
		/****************** Expr_Cosine ******************/
		%feature("compactdefaultargs") Expr_Cosine;
		%feature("autodoc", "Creates the cosine of exp.

Parameters
----------
Exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Cosine;
		 Expr_Cosine(const opencascade::handle<Expr_GeneralExpression> & Exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Cosine)

%extend Expr_Cosine {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/************************
* class Expr_Difference *
************************/
class Expr_Difference : public Expr_BinaryExpression {
	public:
		/****************** Expr_Difference ******************/
		%feature("compactdefaultargs") Expr_Difference;
		%feature("autodoc", "Creates the difference <exp1> - <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_Difference;
		 Expr_Difference(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** NDerivative ******************/
		%feature("compactdefaultargs") NDerivative;
		%feature("autodoc", "Returns the <n>-th derivative on <x> unknown of <self>. raises outofrange if <n> <= 0.

Parameters
----------
X: Expr_NamedUnknown
N: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") NDerivative;
		virtual opencascade::handle<Expr_GeneralExpression> NDerivative(const opencascade::handle<Expr_NamedUnknown> & X, const Standard_Integer N);

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Difference)

%extend Expr_Difference {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/***********************
* class Expr_Different *
***********************/
class Expr_Different : public Expr_SingleRelation {
	public:
		/****************** Expr_Different ******************/
		%feature("compactdefaultargs") Expr_Different;
		%feature("autodoc", "Creates the relation <exp1> # <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_Different;
		 Expr_Different(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsSatisfied;
		Standard_Boolean IsSatisfied();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by associated expressions, and computes values in <self>.

Returns
-------
None
") Simplify;
		void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Different)

%extend Expr_Different {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/**********************
* class Expr_Division *
**********************/
class Expr_Division : public Expr_BinaryExpression {
	public:
		/****************** Expr_Division ******************/
		%feature("compactdefaultargs") Expr_Division;
		%feature("autodoc", "Creates the division <exp1>/<exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_Division;
		 Expr_Division(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Division)

%extend Expr_Division {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*******************
* class Expr_Equal *
*******************/
class Expr_Equal : public Expr_SingleRelation {
	public:
		/****************** Expr_Equal ******************/
		%feature("compactdefaultargs") Expr_Equal;
		%feature("autodoc", "Creates the relation <exp1> = <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_Equal;
		 Expr_Equal(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsSatisfied;
		Standard_Boolean IsSatisfied();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by an associated expressions and computes values in <self>.

Returns
-------
None
") Simplify;
		void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Equal)

%extend Expr_Equal {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*************************
* class Expr_Exponential *
*************************/
class Expr_Exponential : public Expr_UnaryExpression {
	public:
		/****************** Expr_Exponential ******************/
		%feature("compactdefaultargs") Expr_Exponential;
		%feature("autodoc", "Creates the exponential of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Exponential;
		 Expr_Exponential(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Exponential)

%extend Expr_Exponential {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/**************************
* class Expr_Exponentiate *
**************************/
class Expr_Exponentiate : public Expr_BinaryExpression {
	public:
		/****************** Expr_Exponentiate ******************/
		%feature("compactdefaultargs") Expr_Exponentiate;
		%feature("autodoc", "Creates the exponential <exp1> ^ <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_Exponentiate;
		 Expr_Exponentiate(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Exponentiate)

%extend Expr_Exponentiate {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*************************
* class Expr_GreaterThan *
*************************/
class Expr_GreaterThan : public Expr_SingleRelation {
	public:
		/****************** Expr_GreaterThan ******************/
		%feature("compactdefaultargs") Expr_GreaterThan;
		%feature("autodoc", "Creates the relation <exp1> > <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_GreaterThan;
		 Expr_GreaterThan(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsSatisfied;
		Standard_Boolean IsSatisfied();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by associated expressions, and computes values in <self>.

Returns
-------
None
") Simplify;
		void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_GreaterThan)

%extend Expr_GreaterThan {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/********************************
* class Expr_GreaterThanOrEqual *
********************************/
class Expr_GreaterThanOrEqual : public Expr_SingleRelation {
	public:
		/****************** Expr_GreaterThanOrEqual ******************/
		%feature("compactdefaultargs") Expr_GreaterThanOrEqual;
		%feature("autodoc", "Creates the relation <exp1> >= <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_GreaterThanOrEqual;
		 Expr_GreaterThanOrEqual(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsSatisfied;
		Standard_Boolean IsSatisfied();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by associated expressions, and computes values in <self>.

Returns
-------
None
") Simplify;
		void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_GreaterThanOrEqual)

%extend Expr_GreaterThanOrEqual {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/**********************
* class Expr_LessThan *
**********************/
class Expr_LessThan : public Expr_SingleRelation {
	public:
		/****************** Expr_LessThan ******************/
		%feature("compactdefaultargs") Expr_LessThan;
		%feature("autodoc", "Creates the relation <exp1> < <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_LessThan;
		 Expr_LessThan(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsSatisfied;
		Standard_Boolean IsSatisfied();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by associated expressions, and computes values in <self>.

Returns
-------
None
") Simplify;
		void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_LessThan)

%extend Expr_LessThan {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*****************************
* class Expr_LessThanOrEqual *
*****************************/
class Expr_LessThanOrEqual : public Expr_SingleRelation {
	public:
		/****************** Expr_LessThanOrEqual ******************/
		%feature("compactdefaultargs") Expr_LessThanOrEqual;
		%feature("autodoc", "Creates the relation <exp1> <= <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_LessThanOrEqual;
		 Expr_LessThanOrEqual(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Copy;
		opencascade::handle<Expr_GeneralRelation> Copy();

		/****************** IsSatisfied ******************/
		%feature("compactdefaultargs") IsSatisfied;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsSatisfied;
		Standard_Boolean IsSatisfied();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalrelation after replacement of namedunknowns by an associated expression, and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralRelation>
") Simplified;
		opencascade::handle<Expr_GeneralRelation> Simplified();

		/****************** Simplify ******************/
		%feature("compactdefaultargs") Simplify;
		%feature("autodoc", "Replaces namedunknowns by associated expressions, and computes values in <self>.

Returns
-------
None
") Simplify;
		void Simplify();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_LessThanOrEqual)

%extend Expr_LessThanOrEqual {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*********************
* class Expr_LogOf10 *
*********************/
class Expr_LogOf10 : public Expr_UnaryExpression {
	public:
		/****************** Expr_LogOf10 ******************/
		%feature("compactdefaultargs") Expr_LogOf10;
		%feature("autodoc", "Creates the base 10 logarithm of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_LogOf10;
		 Expr_LogOf10(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_LogOf10)

%extend Expr_LogOf10 {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/********************
* class Expr_LogOfe *
********************/
class Expr_LogOfe : public Expr_UnaryExpression {
	public:
		/****************** Expr_LogOfe ******************/
		%feature("compactdefaultargs") Expr_LogOfe;
		%feature("autodoc", "Creates the natural logarithm of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_LogOfe;
		 Expr_LogOfe(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_LogOfe)

%extend Expr_LogOfe {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/***************************
* class Expr_NamedConstant *
***************************/
class Expr_NamedConstant : public Expr_NamedExpression {
	public:
		/****************** Expr_NamedConstant ******************/
		%feature("compactdefaultargs") Expr_NamedConstant;
		%feature("autodoc", "Creates a constant value of name <name> and value <value>.

Parameters
----------
name: TCollection_AsciiString
value: float

Returns
-------
None
") Expr_NamedConstant;
		 Expr_NamedConstant(const TCollection_AsciiString & name, const Standard_Real value);

		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <exp> is contained in <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** ContainsUnknowns ******************/
		%feature("compactdefaultargs") ContainsUnknowns;
		%feature("autodoc", "Tests if <self> contains namedunknown. (returns always false).

Returns
-------
bool
") ContainsUnknowns;
		Standard_Boolean ContainsUnknowns();

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** GetValue ******************/
		%feature("compactdefaultargs") GetValue;
		%feature("autodoc", "No available documentation.

Returns
-------
float
") GetValue;
		Standard_Real GetValue();

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** NDerivative ******************/
		%feature("compactdefaultargs") NDerivative;
		%feature("autodoc", "Returns the <n>-th derivative on <x> unknown of <self>. raises outofrange if <n> <= 0.

Parameters
----------
X: Expr_NamedUnknown
N: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") NDerivative;
		virtual opencascade::handle<Expr_GeneralExpression> NDerivative(const opencascade::handle<Expr_NamedUnknown> & X, const Standard_Integer N);

		/****************** NbSubExpressions ******************/
		%feature("compactdefaultargs") NbSubExpressions;
		%feature("autodoc", "Returns the number of sub-expressions contained in <self> (always returns zero).

Returns
-------
int
") NbSubExpressions;
		Standard_Integer NbSubExpressions();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalexpression after replacement of namedunknowns by an associated expression and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Simplified;
		opencascade::handle<Expr_GeneralExpression> Simplified();

		/****************** SubExpression ******************/
		%feature("compactdefaultargs") SubExpression;
		%feature("autodoc", "Returns the <i>-th sub-expression of <self> raises outofrange.

Parameters
----------
I: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SubExpression;
		const opencascade::handle<Expr_GeneralExpression> & SubExpression(const Standard_Integer I);

};


%make_alias(Expr_NamedConstant)

%extend Expr_NamedConstant {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/**************************
* class Expr_NamedUnknown *
**************************/
class Expr_NamedUnknown : public Expr_NamedExpression {
	public:
		/****************** Expr_NamedUnknown ******************/
		%feature("compactdefaultargs") Expr_NamedUnknown;
		%feature("autodoc", "No available documentation.

Parameters
----------
name: TCollection_AsciiString

Returns
-------
None
") Expr_NamedUnknown;
		 Expr_NamedUnknown(const TCollection_AsciiString & name);

		/****************** Assign ******************/
		%feature("compactdefaultargs") Assign;
		%feature("autodoc", "Assigns <self> to <exp> expression. raises exception if <exp> refers to <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Assign;
		void Assign(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** AssignedExpression ******************/
		%feature("compactdefaultargs") AssignedExpression;
		%feature("autodoc", "If exists, returns the assigned expression. an exception is raised if the expression does not exist.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") AssignedExpression;
		const opencascade::handle<Expr_GeneralExpression> & AssignedExpression();

		/****************** Contains ******************/
		%feature("compactdefaultargs") Contains;
		%feature("autodoc", "Tests if <exp> is contained in <self>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
bool
") Contains;
		Standard_Boolean Contains(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** ContainsUnknowns ******************/
		%feature("compactdefaultargs") ContainsUnknowns;
		%feature("autodoc", "Tests if <self> contains namedunknown.

Returns
-------
bool
") ContainsUnknowns;
		Standard_Boolean ContainsUnknowns();

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Deassign ******************/
		%feature("compactdefaultargs") Deassign;
		%feature("autodoc", "Supresses the assigned expression.

Returns
-------
None
") Deassign;
		void Deassign();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsAssigned ******************/
		%feature("compactdefaultargs") IsAssigned;
		%feature("autodoc", "Tests if an expression is assigned to <self>.

Returns
-------
bool
") IsAssigned;
		Standard_Boolean IsAssigned();

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** NbSubExpressions ******************/
		%feature("compactdefaultargs") NbSubExpressions;
		%feature("autodoc", "Returns the number of sub-expressions contained in <self> ( >= 0).

Returns
-------
int
") NbSubExpressions;
		Standard_Integer NbSubExpressions();

		/****************** Replace ******************/
		%feature("compactdefaultargs") Replace;
		%feature("autodoc", "Replaces all occurences of <var> with <with> in <self> raises invalidoperand if <with> contains <self>.

Parameters
----------
var: Expr_NamedUnknown
with: Expr_GeneralExpression

Returns
-------
None
") Replace;
		void Replace(const opencascade::handle<Expr_NamedUnknown> & var, const opencascade::handle<Expr_GeneralExpression> & with);

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** Simplified ******************/
		%feature("compactdefaultargs") Simplified;
		%feature("autodoc", "Returns a generalexpression after replacement of namedunknowns by an associated expression and after values computation.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Simplified;
		opencascade::handle<Expr_GeneralExpression> Simplified();

		/****************** SubExpression ******************/
		%feature("compactdefaultargs") SubExpression;
		%feature("autodoc", "Returns the <i>-th sub-expression of <self> raises outofrange if <i> > nbsubexpressions(me).

Parameters
----------
I: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") SubExpression;
		const opencascade::handle<Expr_GeneralExpression> & SubExpression(const Standard_Integer I);

};


%make_alias(Expr_NamedUnknown)

%extend Expr_NamedUnknown {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/**************************
* class Expr_PolyFunction *
**************************/
class Expr_PolyFunction : public Expr_PolyExpression {
	public:
		/****************** Expr_PolyFunction ******************/
		%feature("compactdefaultargs") Expr_PolyFunction;
		%feature("autodoc", "Creates <self> as <func>(<exps_1>,<exps_2>,...,<exps_n>).

Parameters
----------
func: Expr_GeneralFunction
exps: Expr_Array1OfGeneralExpression

Returns
-------
None
") Expr_PolyFunction;
		 Expr_PolyFunction(const opencascade::handle<Expr_GeneralFunction> & func, const Expr_Array1OfGeneralExpression & exps);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** Function ******************/
		%feature("compactdefaultargs") Function;
		%feature("autodoc", "Returns the function defining <self>.

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Function;
		opencascade::handle<Expr_GeneralFunction> Function();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_PolyFunction)

%extend Expr_PolyFunction {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*********************
* class Expr_Product *
*********************/
class Expr_Product : public Expr_PolyExpression {
	public:
		/****************** Expr_Product ******************/
		%feature("compactdefaultargs") Expr_Product;
		%feature("autodoc", "Creates the product of all members of sequence <exps>.

Parameters
----------
exps: Expr_SequenceOfGeneralExpression

Returns
-------
None
") Expr_Product;
		 Expr_Product(const Expr_SequenceOfGeneralExpression & exps);

		/****************** Expr_Product ******************/
		%feature("compactdefaultargs") Expr_Product;
		%feature("autodoc", "Creates the product of <exp1> and <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_Product;
		 Expr_Product(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Product)

%extend Expr_Product {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/******************
* class Expr_Sign *
******************/
/******************
* class Expr_Sine *
******************/
class Expr_Sine : public Expr_UnaryExpression {
	public:
		/****************** Expr_Sine ******************/
		%feature("compactdefaultargs") Expr_Sine;
		%feature("autodoc", "Creates the sine of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Sine;
		 Expr_Sine(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Sine)

%extend Expr_Sine {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/******************
* class Expr_Sinh *
******************/
class Expr_Sinh : public Expr_UnaryExpression {
	public:
		/****************** Expr_Sinh ******************/
		%feature("compactdefaultargs") Expr_Sinh;
		%feature("autodoc", "Creates the sinh of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Sinh;
		 Expr_Sinh(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Sinh)

%extend Expr_Sinh {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/********************
* class Expr_Square *
********************/
class Expr_Square : public Expr_UnaryExpression {
	public:
		/****************** Expr_Square ******************/
		%feature("compactdefaultargs") Expr_Square;
		%feature("autodoc", "Creates the square of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Square;
		 Expr_Square(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Square)

%extend Expr_Square {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/************************
* class Expr_SquareRoot *
************************/
class Expr_SquareRoot : public Expr_UnaryExpression {
	public:
		/****************** Expr_SquareRoot ******************/
		%feature("compactdefaultargs") Expr_SquareRoot;
		%feature("autodoc", "Creates the square root of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_SquareRoot;
		 Expr_SquareRoot(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_SquareRoot)

%extend Expr_SquareRoot {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*****************
* class Expr_Sum *
*****************/
class Expr_Sum : public Expr_PolyExpression {
	public:
		/****************** Expr_Sum ******************/
		%feature("compactdefaultargs") Expr_Sum;
		%feature("autodoc", "Creates the sum of all the members of sequence <exps>.

Parameters
----------
exps: Expr_SequenceOfGeneralExpression

Returns
-------
None
") Expr_Sum;
		 Expr_Sum(const Expr_SequenceOfGeneralExpression & exps);

		/****************** Expr_Sum ******************/
		%feature("compactdefaultargs") Expr_Sum;
		%feature("autodoc", "Creates the sum of <exp1> and <exp2>.

Parameters
----------
exp1: Expr_GeneralExpression
exp2: Expr_GeneralExpression

Returns
-------
None
") Expr_Sum;
		 Expr_Sum(const opencascade::handle<Expr_GeneralExpression> & exp1, const opencascade::handle<Expr_GeneralExpression> & exp2);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** NDerivative ******************/
		%feature("compactdefaultargs") NDerivative;
		%feature("autodoc", "Returns the <n>-th derivative on <x> unknown of <self>. raises outofrange if <n> <= 0.

Parameters
----------
X: Expr_NamedUnknown
N: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") NDerivative;
		virtual opencascade::handle<Expr_GeneralExpression> NDerivative(const opencascade::handle<Expr_NamedUnknown> & X, const Standard_Integer N);

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Sum)

%extend Expr_Sum {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/*********************
* class Expr_Tangent *
*********************/
class Expr_Tangent : public Expr_UnaryExpression {
	public:
		/****************** Expr_Tangent ******************/
		%feature("compactdefaultargs") Expr_Tangent;
		%feature("autodoc", "Creates the tangent of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Tangent;
		 Expr_Tangent(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Tangent)

%extend Expr_Tangent {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/******************
* class Expr_Tanh *
******************/
class Expr_Tanh : public Expr_UnaryExpression {
	public:
		/****************** Expr_Tanh ******************/
		%feature("compactdefaultargs") Expr_Tanh;
		%feature("autodoc", "Creates the hyperbolic tangent of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_Tanh;
		 Expr_Tanh(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_Tanh)

%extend Expr_Tanh {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/***************************
* class Expr_UnaryFunction *
***************************/
class Expr_UnaryFunction : public Expr_UnaryExpression {
	public:
		/****************** Expr_UnaryFunction ******************/
		%feature("compactdefaultargs") Expr_UnaryFunction;
		%feature("autodoc", "Creates me as <func>(<exp>). raises exception if <func> is not unary.

Parameters
----------
func: Expr_GeneralFunction
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_UnaryFunction;
		 Expr_UnaryFunction(const opencascade::handle<Expr_GeneralFunction> & func, const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** Function ******************/
		%feature("compactdefaultargs") Function;
		%feature("autodoc", "Returns the function defining <self>.

Returns
-------
opencascade::handle<Expr_GeneralFunction>
") Function;
		opencascade::handle<Expr_GeneralFunction> Function();

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_UnaryFunction)

%extend Expr_UnaryFunction {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/************************
* class Expr_UnaryMinus *
************************/
class Expr_UnaryMinus : public Expr_UnaryExpression {
	public:
		/****************** Expr_UnaryMinus ******************/
		%feature("compactdefaultargs") Expr_UnaryMinus;
		%feature("autodoc", "Create the unary minus of <exp>.

Parameters
----------
exp: Expr_GeneralExpression

Returns
-------
None
") Expr_UnaryMinus;
		 Expr_UnaryMinus(const opencascade::handle<Expr_GeneralExpression> & exp);

		/****************** Copy ******************/
		%feature("compactdefaultargs") Copy;
		%feature("autodoc", "Returns a copy of <self> having the same unknowns and functions.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Copy;
		opencascade::handle<Expr_GeneralExpression> Copy();

		/****************** Derivative ******************/
		%feature("compactdefaultargs") Derivative;
		%feature("autodoc", "Returns the derivative on <x> unknown of <self>.

Parameters
----------
X: Expr_NamedUnknown

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") Derivative;
		opencascade::handle<Expr_GeneralExpression> Derivative(const opencascade::handle<Expr_NamedUnknown> & X);

		/****************** Evaluate ******************/
		%feature("compactdefaultargs") Evaluate;
		%feature("autodoc", "Returns the value of <self> (as a real) by replacement of <vars> by <vals>. raises notevaluable if <self> contains namedunknown not in <vars> or numericerror if result cannot be computed.

Parameters
----------
vars: Expr_Array1OfNamedUnknown
vals: TColStd_Array1OfReal

Returns
-------
float
") Evaluate;
		Standard_Real Evaluate(const Expr_Array1OfNamedUnknown & vars, const TColStd_Array1OfReal & vals);

		/****************** IsIdentical ******************/
		%feature("compactdefaultargs") IsIdentical;
		%feature("autodoc", "Tests if <self> and <other> define the same expression. this method does not include any simplification before testing.

Parameters
----------
Other: Expr_GeneralExpression

Returns
-------
bool
") IsIdentical;
		Standard_Boolean IsIdentical(const opencascade::handle<Expr_GeneralExpression> & Other);

		/****************** IsLinear ******************/
		%feature("compactdefaultargs") IsLinear;
		%feature("autodoc", "No available documentation.

Returns
-------
bool
") IsLinear;
		Standard_Boolean IsLinear();

		/****************** NDerivative ******************/
		%feature("compactdefaultargs") NDerivative;
		%feature("autodoc", "Returns the <n>-th derivative on <x> unknown of <self>. raises outofrange if <n> <= 0.

Parameters
----------
X: Expr_NamedUnknown
N: int

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") NDerivative;
		virtual opencascade::handle<Expr_GeneralExpression> NDerivative(const opencascade::handle<Expr_NamedUnknown> & X, const Standard_Integer N);

		/****************** ShallowSimplified ******************/
		%feature("compactdefaultargs") ShallowSimplified;
		%feature("autodoc", "Returns a generalexpression after a simplification of the arguments of <self>.

Returns
-------
opencascade::handle<Expr_GeneralExpression>
") ShallowSimplified;
		opencascade::handle<Expr_GeneralExpression> ShallowSimplified();

		/****************** String ******************/
		%feature("compactdefaultargs") String;
		%feature("autodoc", "Returns a string representing <self> in a readable way.

Returns
-------
TCollection_AsciiString
") String;
		TCollection_AsciiString String();

};


%make_alias(Expr_UnaryMinus)

%extend Expr_UnaryMinus {
	%pythoncode {
	__repr__ = _dumps_object
	}
};

/* python proxy for excluded classes */
%pythoncode {
@classnotwrapped
class Expr_Sign:
	pass

}
/* end python proxy for excluded classes */
/* harray1 classes */
/* harray2 classes */
/* hsequence classes */
