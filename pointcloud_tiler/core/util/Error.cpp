#include "util/Error.h"

#include <sstream>

util::ErrorChain::ErrorChain(const std::exception &reason)
    : _reasons(reason.what()) {}

util::ErrorChain::ErrorChain(std::string reason)
    : _reasons(std::move(reason)) {}

util::ErrorChain::ErrorChain(const ErrorChain &reason, const std::string &why) {
  std::stringstream ss;
  ss << why << "\n\tcaused by: " << reason.what();
  _reasons = ss.str();
}

const char *util::ErrorChain::what() const noexcept { return _reasons.c_str(); }

util::ErrorChain util::chain_error(std::string why) {
  return ErrorChain{std::move(why)};
}

util::ErrorChain util::chain_error(const std::exception &exception,
                                   const std::string &why) {
  return ErrorChain{ErrorChain{exception}, why};
}

util::ErrorChain util::chain_error(const util::ErrorChain &reason,
                                   const std::string &why) {
  return ErrorChain{reason, why};
}