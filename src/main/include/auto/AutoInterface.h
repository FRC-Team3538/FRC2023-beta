#pragma once

#include <iostream>

class AutoInterface
{
public:
  // Periodic task
  virtual void Run() = 0;
  virtual void Init() = 0;
  virtual void UpdateSmartDash() = 0;
  virtual ~AutoInterface(){};
};