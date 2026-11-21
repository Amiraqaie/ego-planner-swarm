#pragma once

#include <memory>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

namespace plan_env
{
template <typename Visitor>
bool visitFullOctomap(const octomap_msgs::msg::Octomap &msg, Visitor visit)
{
  if (msg.binary)
    return false;

  std::unique_ptr<octomap::AbstractOcTree> map(octomap_msgs::fullMsgToMap(msg));
  if (const auto *tree = dynamic_cast<const octomap::OcTree *>(map.get()))
    visit(*tree);
  else if (const auto *tree = dynamic_cast<const octomap::ColorOcTree *>(map.get()))
    visit(*tree);
  else
    return false;
  return true;
}
}  // namespace plan_env
