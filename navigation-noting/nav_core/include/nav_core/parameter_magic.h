
#ifndef NAV_CORE_PARAMETER_MAGIC_H
#define NAV_CORE_PARAMETER_MAGIC_H

namespace nav_core
{

/**
 * @brief Load a parameter from one of two namespaces. Complain if it uses the old name.
 * @param nh NodeHandle to look for the parameter in
 * @param current_name Parameter name that is current, i.e. not deprecated
 * @param old_name Deprecated parameter name
 * @param default_value If neither parameter is present, return this value
 * @return The value of the parameter or the default value
 */
template<class param_t>
param_t loadParameterWithDeprecation(const ros::NodeHandle& nh, const std::string current_name,
                                     const std::string old_name, const param_t& default_value)
{
  param_t value;
  if (nh.hasParam(current_name))
  {
    nh.getParam(current_name, value);
    return value;
  }
  if (nh.hasParam(old_name))
  {
    ROS_WARN("Parameter %s is deprecated. Please use the name %s instead.", old_name.c_str(), current_name.c_str());
    nh.getParam(old_name, value);
    return value;
  }
  return default_value;
}

/**
 * @brief Warn if a parameter exists under a deprecated (and unsupported) name.
 *
 * Parameters loaded exclusively through dynamic reconfigure can't really use loadParamWithDeprecation.
 */
void warnRenamedParameter(const ros::NodeHandle& nh, const std::string current_name, const std::string old_name)
{
  if (nh.hasParam(old_name))
  {
    ROS_WARN("Parameter %s is deprecated (and will not load properly). Use %s instead.", old_name.c_str(), current_name.c_str());
  }
}

}  // namespace nav_core

#endif  // NAV_CORE_PARAMETER_MAGIC_H
