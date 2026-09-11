// Source - https://stackoverflow.com/q/65924627
// Posted by A.Manfreda, modified by community. See post 'Timeline' for change history
// Retrieved 2026-09-11, License - CC BY-SA 4.0

%{
#include <variant>

static PyObject *this_module = NULL;
%}

%init %{
  // We need to "borrow" a reference to this for our typemaps to be able to look up the right functions
  this_module = m; // borrow should be fine since we can only get called when our module is loaded right?
  // Wouldn't it be nice if $module worked *anywhere*
%}

#define FE_0(...)
#define FE_1(action,a1) action(0,a1)
#define FE_2(action,a1,a2) action(0,a1); action(1,a2)
#define FE_3(action,a1,a2,a3) action(0,a1); action(1,a2); action(2,a3)
#define FE_4(action,a1,a2,a3,a4) action(0,a1); action(1,a2); action(2,a3); action(3,a4)
#define FE_5(action,a1,a2,a3,a4,a5) action(0,a1); action(1,a2); action(2,a3); action(3,a4); action(4,a5)

#define GET_MACRO(_1,_2,_3,_4,_5,NAME,...) NAME
%define FOR_EACH(action,...)
  GET_MACRO(__VA_ARGS__, FE_5, FE_4, FE_3, FE_2, FE_1, FE_0)(action,__VA_ARGS__)
%enddef

#define in_helper(num,type) const type & convert_type ## num () { return std::get<type>(*$self); }
#define constructor_helper(num,type) variant(const type&)

%define %std_variant(Name, ...)

%rename(Name) std::variant<__VA_ARGS__>;

namespace std {
  struct variant<__VA_ARGS__> {
    variant();
    variant(const std::variant<__VA_ARGS__>&);
    FOR_EACH(constructor_helper, __VA_ARGS__);
    int index();

    %extend {
      FOR_EACH(in_helper, __VA_ARGS__);
    }
  };
}

%typemap(out) std::variant<__VA_ARGS__> {
  // Make our function output into a PyObject
  PyObject *tmp = SWIG_NewPointerObj(&$1, $&1_descriptor, 0); // Python does not own this object...

  // Pass that temporary PyObject into the helper function and get another PyObject back in exchange
  const std::string func_name = "convert_type" + std::to_string($1.index());
  $result = PyObject_CallMethod(tmp, func_name.c_str(),  "");
  Py_DECREF(tmp);
}

%typemap(in) const std::variant<__VA_ARGS__>& (PyObject *tmp=NULL) {
  // I don't much like having to "guess" the name of the make_variant we want to use here like this...
  // But it's hard to support both -builtin and regular modes and generically find the right code.
  PyObject *helper_func = PyObject_GetAttrString(this_module, "new_" #Name );
  assert(helper_func);
  // TODO: is O right, or should it be N?
  tmp = PyObject_CallFunction(helper_func, "O", $input);
  Py_DECREF(helper_func);
  if (!tmp) SWIG_fail; // An exception is already pending

  // TODO: if we cared, we chould short-circuit things a lot for the case where our input really was a variant object
  const int res = SWIG_ConvertPtr(tmp, (void**)&$1, $1_descriptor, 0);
  if (!SWIG_IsOK(res)) {
    SWIG_exception_fail(SWIG_ArgError(res), "Variant typemap failed, not sure if this can actually happen");
  }
}

%typemap(freearg) const std::variant<__VA_ARGS__>& %{
  Py_DECREF(tmp$argnum);
%}

%enddef
