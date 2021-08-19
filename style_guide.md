# MTRN2500 Style Guide based on ROS 2 Developer Guide


This page defines the practices and policies we will use in MTRN2500.


## General Practices



### Development Process

* The default branch (in most cases the master branch) must always build, pass all tests and compile without warnings.
  If at any time there is a regression it is the top priority to restore at least the previous state.
* Always build with tests enabled.
* Always run tests locally after changes and before proposing them in a pull request.
  Besides using automated tests, also run the modified code path manually to ensure that the patch works as intended.
* Always run CI jobs for all platforms for every pull request and include links to the jobs in the pull request.

For more details on recommended software development workflow, see `Software Development Lifecycle` section.


### Programming conventions

* Defensive programming: ensure that assumptions are held as early as possible.
  E.g. check every return code and make sure to at least throw an exception until the case is handled more gracefully.
* All error messages must be directed to ``stderr``.
* Declare variables in the narrowest scope possible.
* Keep group of items (dependencies, imports, includes, etc.) ordered alphabetically.

### Language Versions and Code Format


In order to achieve a consistent looking product we will follow this style guide.

Additionally, where ever possible, developers should use integrated tools to allow them to check that these guidelines are followed in their editors.

Also where possible, packages should check style as part of their unit tests to help with the automated detection of style issues (see [ament_lint_auto](https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst)). A clangformat sheet is provided. 

### Style
* In case of conflict with this style guide, prefer the provided clang-format setting.
We will use the [ROS 2 Developer Guide](https://index.ros.org//doc/ros2/Contributing/Developer-Guide/) which is based on [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) , with some modifications. You should read those documents.

### Standard
This course will target C++17*, plus format library from C++20 (*note ```<filesystem>``` and ```<charconv>``` are not implemented in gcc 7.4).

ROS2 target C++14, using new built-in C++14 features over Boost equivalents where ever possible.


## Forbidden features
The following language features are banned because either better alternative exist in c++ or they are generally considered dangerous.
* Goto: make controlflow difficult to analyse.
* Global variables (except constexpr): can cause hard to find bug. See [CppCoreGuidelines I.2: Avoid non-const global variables](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Ri-global)
* C style Cast: use c++ ```static_cast<>``` instead for most usecase, on rare occation ```reinterpret_cast<>``` is required
* raw new/delete: dangerous, use smart pointer ```std::unique_ptr``` instead, occationally ```std::shared_ptr<>```
* using namespace std: See [CppCoreGuidelines SF.6: Use using namespace directives for transition, for foundation libraries...](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rs-using)
* C macro: Macro is just dumb text replacement that can cause unexpected result. Use constexpr variable for magic constants instead. See [cppCoreGuidelines ES.45: Avoid "magic constants"; use symbolic constants](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Res-magic)
* Using namespace in global scope in a header. Doing so may cause unpredicatble name collision with user's code. [cppCoreGuidelines SF.7: Don't write using namespace at global scope in a header file](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rs-using-directive)
* double underscore __ or leading underscore in names. Most of them are reserved by the standard. 

### Line Length
* Our maximum line length is **80** characters. 

### Variable Naming

* Class name should use ``CamelCase``. For everything else, follow C++ std library's style of using ``snake_case``.

* Use meaningful names. Name should convey all the useful infomation the programmer need. [Naming is hard here is a prep talk](https://www.youtube.com/watch?v=MBRoCdtZOYg) [Slides](https://github.com/CppCon/CppCon2019/raw/master/Presentations/naming_is_hard_lets_do_better/naming_is_hard_lets_do_better__kate_gregory__cppcon_2019.pdf)

* Classes and objects represent things, they should be named like a noun.
* Functions and methods preform actions, they should be named like a verb.
    
### Access Control

* Drop requirement for all class members to be private and therefore require accessors

  * rationale: this is overly constraining for user API design
  * we should **prefer private members, only making them public when they are needed**
  * we should consider using accessors before choosing to allow direct member access
  * we should have a good reason for allowing direct member access, other than because it is convenient for us

### Exceptions

* Exceptions are allowed

  * rationale: this is a new code base, so the legacy argument doesn't apply to us
  * rationale: for user facing API's it is more idiomatic C++ to have exceptions
  * Exceptions in destructors should be explicitly avoided

* We should consider avoiding Exceptions if we intend to wrap the resulting API in C

  * rationale: it will make it easier to wrap in C
  * rationale: most of our dependencies in code we intend to wrap in C do not use exceptions anyways

###  Function-like Objects

* No restrictions on Lambda's or ``std::function`` or ``std::bind``

### Boost

* Boost should be avoided until absolutely required

### Comments and Doc Comments

* Use ``///`` and ``/** */`` comments for *documentation* purposes and ``//`` style comments for notes and general comments

  * Class and Function comments should use ``///`` and ``/** */`` style comments
  * rationale: these are recommended for Doxygen and Sphinx in C/C++
  * rationale: mixing ``/* */`` and ``//`` is convenient for block commenting out code which contains comments
  * Descriptions of how the code works or notes within classes and functions should use ``//`` style comments

###  Pointer Syntax Alignment

* Use ``char * c;`` instead of ``char* c;`` or ``char *c;`` because of this scenario ``char* c, *d, *e;``

### Class Privacy Keywords
* Do not put 1 space before ``public:``, ``private:``, or ``protected:``, it is more consistent for all indentions to be a multiple of 2

  * rationale: most editors don't like indentions which are not a multiple of the (soft) tab size
  * Use zero spaces before ``public:``, ``private:``, or ``protected:``, or 2 spaces
  * If you use 2 spaces before, indent other class statements by 2 additional spaces
  * Prefer zero spaces, i.e. ``public:``, ``private:``, or ``protected:`` in the same column as the class

### Nested Templates
* Never add whitespace to nested templates

  * Prefer ``set<list<string>>`` (C++11 feature) to ``set<list<string> >`` or ``set< list<string> >``

### Always Use Braces
* Always use braces following ``if``, ``else``, ``do``, ``while``, and ``for``, even when the body is a single line.

  * rationale: less opportunity for visual ambiguity and for complications due to use of macros in the body


### Open Versus Cuddled Braces

* Use open braces for ``function``, ``class``, and ``struct`` definitions, and ``if``, ``else``, ``while``, ``for``, etc...
  * Reasoning for modification: Simpler.
  * Open braces create white space that breaks code into sections similar to paragraphs.

* When a function call cannot fit on one line, wrap at the open parenthesis (not in between arguments) and start them on the next line with an indent.  Continue with the indent on subsequent lines for more arguments.  (Note that the `Google style guide <https://google.github.io/styleguide/cppguide.html#Function_Calls>`__ is internally contradictory on this point.)

  * Same goes for ``if`` (and ``while``, etc.) conditions that are too long to fit on one line.
  
  * Same goes member initializer lists that are too long to fit on one line.

### Examples

This is OK:

```c++

   int main(int argc, char **argv)
   {
     if (condition)
     {
       return 0;
     } else {
       return 1;
     }
   }
```

### Linters

Most of these styles and restrictions can be checked with a combination of Google's `cpplint.py <http://google-styleguide.googlecode.com/svn/trunk/cpplint/>`__ and `uncrustify <https://github.com/uncrustify/uncrustify>`__, though we may need to modify them slightly for our above changes.

We provide command line tools with custom configurations:

* `ament_cpplint <https://github.com/ament/ament_lint/blob/master/ament_cpplint/doc/index.rst>`__
* `ament_uncrustify <https://github.com/ament/ament_lint/blob/master/ament_uncrustify/doc/index.rst>`__: `configuration <https://github.com/ament/ament_lint/blob/master/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg>`__

We also run other tools to detect and eliminate as many warnings as possible.
Here's a non-exhaustive list of additional things we try to do on all of our packages:

* use compiler flags like ``-Wall -Wextra -Wpedantic``
* run static code analysis like ``cppcheck``, which we have integrated in `ament_cppcheck <https://github.com/ament/ament_lint/blob/master/ament_cppcheck/doc/index.rst>`__.

## Testing

All packages should have some level of tests.
Tests can be broken down into three main categories: System tests, Integration tests, and Unit tests.

Unit tests should always be in the package which is being tested and should make use of tools like ``Mock`` to try and test narrow parts of the code base in constructed scenarios.
Unit tests should not bring in test dependencies that are not testing tools, e.g. gtest, nosetest, pytest, mock, etc...

Integration tests can test interactions between parts of the code or between parts of the code and the system.
They often test software interfaces in ways that we expect the user to use them.
Like Unit tests, Integration tests should be in the package which is being tested and should not bring in non-tool test dependencies unless absolutely necessary, i.e. all non-tool dependencies should only be allowed under extreme scrutiny so they should be avoided if possible.

System tests are designed to test end-to-end situations between packages and should be in their own packages to avoid bloating or coupling packages and to avoid circular dependencies.

In general minimizing external or cross package test dependencies should be avoided to prevent circular dependencies and tightly coupled test packages.

All packages should have some unit tests and possibly integration tests, but the degree to which they should have them is based on the package's category (described later).

### Test Coverage

Some packages should have a mechanism setup to capture test coverage information (if applicable to the language).
Coverage tools exist for some of the languages described here including C, C++, and Python, but possibly others.
When possible coverage should be measured in terms of branch coverage, as opposed to statement or function coverage.


## Filesystem Layout


The filesystem layout of packages and repositories should follow the same conventions in order to provide a consistent experience for users browsing our source code.

### Package layout

* ``src``: contains all C and C++ code

  * Also contains C/C++ headers which are not installed

* ``include``: contains all C and C++ headers which are installed

* ``test``: contains all automated tests and test data
* ``doc``: contains all the documentation
* ``package.xml``: as defined by [REP-0140](http://www.ros.org/reps/rep-0140.html) (may be updated for prototyping)
* ``CMakeLists.txt``: only ROS packages which use CMake
* ``README``: README which can be rendered on Github as a landing page for the project

  * This can be as short or detailed as is convenient, but it should at least link to project documentation
  * Consider putting a CI or code coverage tag in this readme
  * It can also be ``.rst`` or anything else that Github supports

* ``LICENSE``: A copy of the license or licenses for this package
* ``CHANGELOG.rst``: [REP-0132](http://www.ros.org/reps/rep-0132.html) compliant changelog

## Repository layout

Each package should be in a subfolder which has the same name as the package.
If a repository contains only a single package it can optionally be in the root of the repository.

The root of the repository should have a ``CONTRIBUTING`` file describing the contribution guidelines.
This might include license implication when using e.g. the Apache 2 License.

## Documentation

*(API docs are not yet being automatically generated)*

All packages should have these documentation elements:

* Description and purpose
* Definition and description of the public API
* Examples
* How to build and install (should reference external tools/workflows)
* How to build and run tests
* How to build documentation
* How to develop (useful for describing things like ``python setup.py develop``)

Each package should describe itself and its purpose or how it is used in the larger scope.
The description should be written, as much as possible, assuming that the reader has stumbled onto it without previous knowledge of ROS or other related projects.

Each package should define and describe its public API so that there is a reasonable expectation for users what is covered by the semantic versioning policy.
Even in C and C++, where the public API can be enforced by API and ABI checking, it is a good opportunity to describe the layout of the code and the function of each part of the code.

It should be easy to take any package and from that package's documentation understand how to build, run, build and run tests, and build the documentation.
Obviously we should avoid repeating ourselves for common workflows, like build a package in a workspace, but the basic workflows should be either described or referenced.

Finally, it should include any documentation for developers.
This might include workflows for testing the code using something like ``python setup.py develop``, or it might mean describing how to make use of extension points provided by you package.

Examples:

* capabilities: http://docs.ros.org/hydro/api/capabilities/html/

  * This one gives an example of docs which describe the public API

* catkin_tools: https://catkin-tools.readthedocs.org/en/latest/development/extending_the_catkin_command.html

  * This is an example of describing an extension point for a package