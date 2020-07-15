# Contributing to RMF

Contributions via pull requests to the various RMF-related repositories are welcome.
In general, we expect contributors to follow the [ROS 2 contributing guidelines](https://index.ros.org/doc/ros2/Contributing/), with the exception of code style.
The RMF code adopts slightly different guidelines from the [ROS 2 style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/#codestyle).
The objective of this document is to highlight the divergences of RMF code style from the ROS 2 style guide for C++.

## RMF Code Style 

### C++
#### Standard

- The `eloquent` release uses C++14
- The `foxy` release uses C++17

#### Style

**Line Length**

* Maximum line length is 80 characters.

**File Naming and Extensions**
  * Original files with public interfaces should follow `CamelCase` convention
  * Header files should use the `.hpp` extension and must include header guards
  * Implementation files should use the `.cpp` extension.

**Braces**
  * Use open braces without indenting the braces for function, class, and struct definitions and on if, else, while, for, etc…
  * Cuddle braces on namespace definitions only

**Indentations**
  * Two-space indentation per level
  * Two-space continued indentation for function definition/call parameters

**Variable Naming**
  * All variable names should use `snake_case`
  * Private member variables should be prefixed with an underscore `_`

**Function and Method Naming**
  * All function names including class member functions should use `snake_case`

**Classes**
  * Class names should always use `CamelCase`
  * Privacy specifiers (`public`, `private`, `protected`) should not be indented
  * Only member functions are allowed in `public` scope in public APIs (no `public` data members)
  * Two-space indentation for other class statements
  * Leading colon between a constructor and its member initialization list
  * No space/indent for constructor initialization lists
  * Trailing commas between members in the initialization lists
  * Do not use `struct` in public APIs. Usage in internal implementation is allowed.
  * Abstract interface classes should contain only pure abstract member functions. No data fields or function implementations are allowed



**Namespaces**
* Cuddle brace for namespaces with a space between the name and the opening brace
* No indentation for namespace contents (including nested namespaces)
* Closing brace to be followed with a comment with the namespace name, e.g,
```
namespace A {

/* code */

} // namespace A
```

**Pointer and Reference Syntax Alignment**
  * Left-align `*` and `&`

**Comments and Doc Comments**
  * Use /// and /** */ comments for documentation purposes and // style comments for notes and general comments

**Examples**

Description of a class
```
namespace A {
namespace B {
class UpperCamelCase
{
public:

  UpperCamelCase(Foo foo, Bar bar)
  : _foo(foo),
    _bar(bar),
    _baz(Baz(foo, bar))
  {
  }

  /// This is an example function
  ///
  /// And here is a longer description blah blah blah
  ///
  /// \param[in] in_value
  ///   It takes in a value
  ///
  /// \param[out] out_value
  ///   It puts out a value
  ///
  /// \return some result
  ResultType snake_case_member_functions(
    InputValue in_value,
    OutputValue& out_value) const;

private:
  /* ... etc ... */
};

} // namespace B
} // namespace A

```

A class that is only used internally

```
class ImplementationClass
{
public:

  /// documentation
  double snake_case_public_members_uncommon;

  void foo(Bar bar);

private:

  int _lead_with_underscore;

};
```

A class that defines an interface

```

class AbstractInterfaceClass
{
public:

  /// Only pure abstract member functions.
  /// No data fields or implemented
  ///functions
  virtual ReturnType pure_virtual_function(
    SomeArgType arg1,
    SomeArgType arg2) = 0;

};
```

**RMF Linter**

Most of these styles and restrictions can be checked with the `ament_uncrustify` command line tool using [this configuration file](https://github.com/osrf/rmf_core/blob/master/rmf_utils/test/format/rmf_code_style.cfg).

Example usage:
```
cd workspace/
wget https://raw.githubusercontent.com/osrf/rmf_core/master/rmf_utils/test/format/rmf_code_style.cfg
source /opt/ros/foxy/setup.bash
ament_uncrustify -c rmf_code_style.cfg .
```
The `--reformat` option may be passed into the `ament_uncrustify` call to apply the changes in place. However, this is recommended only after manually reviewing the changes.
