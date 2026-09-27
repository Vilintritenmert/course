#pragma once

#include <exception>
#include <string>

namespace uav
{

  class UavException : public std::exception
  {
  public:
    explicit UavException(std::string message) : message_(std::move(message)) {}

    auto what() const noexcept -> const char * override { return message_.c_str(); }

  private:
    std::string message_;
  };

}
