#include "util/Config.h"

TilerConfig&
global_config()
{
  static TilerConfig s_config;
  return s_config;
}