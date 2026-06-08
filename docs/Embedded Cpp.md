# Embedded C++

Writing C++ for embedded systems requires different software design patterns than typical applications. A few styles used in PSP AC's codebases are described below, along with justification.

## Use Clear and Concise Variable and Function Names

Variables and functions should have the shortest name that makes their use unambiguous. If not obvious, including units in the variable name (`wait_time_ms` instead of `wait_time`) reduces the chance of mistakes. Functions names should generally be verbs / actions (`get_last_packet()` instead of `last_packet()`). Function names can rely on the namespace they are in (`PressureSenors::calibrate()` is fine, `PressureSensors::calibrate_pressure_sensors()` is unecessary). 

## Minimize Lines of Code

As a general rule, write less code. Less code makes changes easier to review and limits the places bugs can be hiding in the codebase. Be straightforward and avoid layers of function calls. Repeated patterns should be broken out into functions, but its fine to have 2 similar sections of code if the subtle differences would make a shared function complex.

## Mostly Write C

C++ is incredibly powerful, but with great power comes great complexity. PSP AC's codebases stick to C features with the exception of namespaces and the occasional use of classes. This includes using `printf` instead of the C++ stream operators, and (generally) using C structs instead of C++ classes.

## Avoid Memory Allocation

Because we work on limited hardware, we should know how much memory is needed when the program starts. Occasionally dynamic allocation will be needed during initialization (`begin()` calls), but should never occur during the flight loop. Memory allocation is prone to mistakes, such as memory leaks and use-after- frees. Additionally, memory allocation can be slow (think of the complexity inside `malloc()`) and the performance is unpredictable. See [NASA's coding standards](https://spinroot.com/gerard/pdf/P10.pdf), rule 3.

