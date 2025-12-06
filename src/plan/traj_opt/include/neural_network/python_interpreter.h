#pragma once

#include <pybind11/embed.h>

#include <iostream>

namespace py = pybind11;

/*
  This head file is used to initialize the python interpreter.
  workspace probability model and forward model share the same python interpreter.
  so we need to initialize the interpreter once and only once.
*/
namespace neural_network
{
class PythonInterpreter
{
 private:
  static bool initialized;

 public:
  static bool initialize()
  {
    if (initialized) return true;

    try
    {
      static py::scoped_interpreter guard{};
      initialized = true;
      return true;
    }
    catch (const std::exception& e)
    {
      std::cerr << "[PythonInterpreter_H]: Error initializing Python interpreter: "
                << e.what() << std::endl;
      return false;
    }
  }
};

bool PythonInterpreter::initialized = false;

}  // namespace neural_network