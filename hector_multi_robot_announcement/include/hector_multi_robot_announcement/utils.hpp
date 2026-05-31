#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP

#include <string>

namespace hector_multi_robot_announcement
{

//! @brief Normalizes a ROS namespace to its canonical absolute form: a single leading slash,
//!        no repeated slashes, and no trailing slash (root stays "/"). Empty input maps to "/".
//!
//! A trailing or repeated slash would otherwise survive into a composed topic name (e.g.
//! "<ns>/tf" becoming "/robot1//tf"), which rmw rejects, throwing out of node construction.
inline std::string normalize_namespace( const std::string &ros_namespace )
{
  std::string result = "/";
  for ( const char c : ros_namespace ) {
    if ( c == '/' && result.back() == '/' )
      continue; // collapse leading and repeated slashes into the single leading "/"
    result.push_back( c );
  }
  if ( result.size() > 1 && result.back() == '/' )
    result.pop_back(); // drop trailing slash (root stays "/")
  return result;
}

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP
