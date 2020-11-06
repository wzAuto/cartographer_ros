#include <ros/ros.h>

#include <iostream>
#include <set>

#include "cartographer_ros_msgs/ReadMetrics.h"
#include "node_constants.h"

const std::set<std::string> display_metrics{
    "mapping_constraints_constraint_builder_2d_constraints",
    "mapping_global_trajectory_builder_local_slam_results",
    "mapping_constraints_constraint_builder_2d_queue_length",
    "mapping_constraints_constraint_builder_2d_num_submap_scan_matchers",
    "mapping_2d_local_trajectory_builder_latency",
    "mapping_2d_local_trajectory_builder_real_time_ratio",
    "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
    "mapping_2d_pose_graph_constraints",
    "mapping_2d_pose_graph_submaps",
    "mapping_constraints_constraint_builder_2d_scores",
    "mapping_2d_local_trajectory_builder_scores",
    "mapping_2d_local_trajectory_builder_costs",
    "mapping_2d_local_trajectory_builder_residuals"};

int main(int argc, char** argv) {
  ros::init(argc, argv, "metrics_test");

  ros::NodeHandle node;

  // client metric
  ros::ServiceClient client =
      node.serviceClient<cartographer_ros_msgs::ReadMetrics>(
          cartographer_ros::kReadMetricsServiceName);

  cartographer_ros_msgs::ReadMetrics srv_metric;
  ros::Rate rate(20.0);
  while (ros::ok()) {
    if (client.call(srv_metric)) {
      std::cout << "srv_metric.response.status.message: "
                << srv_metric.response.status.message << std::endl;

      for (const auto& family_msg : srv_metric.response.metric_families) {
        // auto exist = display_metrics.find(family_msg.name);
        // if (exist == display_metrics.end()) {
        //   continue;
        // }
        std::cout << "metric_name: " << family_msg.name << std::endl;
        for (const auto& score_msg : family_msg.metrics) {
          // std::cout << "score_msg.labels: " << score_msg.labels.size()
          //           << std::endl;
          if (not score_msg.labels.empty()) {
            const auto& label = score_msg.labels.front();
            std::cout << "score_msg key, value: " << label.key << ", "
                      << label.value << std::endl;
          }

          switch (score_msg.type) {
            case cartographer_ros_msgs::Metric::TYPE_HISTOGRAM:
              // std::cout << "histogram: " << score_msg.value << std::endl;
              for (const auto buck : score_msg.counts_by_bucket) {
                if (buck.count > 0) {
                  std::cout << "histogram boundary, count: ("
                            << buck.bucket_boundary << ", " << buck.count << ")"
                            << std::endl;
                }
              }
              break;
            case cartographer_ros_msgs::Metric::TYPE_GAUGE:
              std::cout << "gauge    : " << score_msg.value << std::endl;
              break;
            case cartographer_ros_msgs::Metric::TYPE_COUNTER:
              std::cout << "counter  : " << score_msg.value << std::endl;
              break;
          }
        }
      }
    }

    rate.sleep();
  }
  return 0;
}