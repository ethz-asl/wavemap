#include <sstream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/dense_grid.h"
#include "wavemap_2d/data_structure/volumetric/simple_quadtree.h"
#include "wavemap_2d/transform/dense/lifted_cdf_5_3.h"
#include "wavemap_2d/transform/dense/lifted_cdf_9_7.h"
#include "wavemap_2d/transform/dense/naive_haar.h"
#include "wavemap_2d/transform/tree/child_averaging.h"
#include "wavemap_2d/utils/evaluation_utils.h"

DEFINE_string(estimated_map_file_path, "", "Path to the estimated map.");
DEFINE_double(estimated_map_min_cell_width, 0.01,
              "Resolution of the estimated map in meters.");
DEFINE_bool(
    estimated_map_saved_with_floating_precision, true,
    "Whether the estimated map was saved with floating point precision.");
DEFINE_string(ground_truth_map_file_path, "",
              "Path to the ground truth map (e.g. generated with "
              "wavemap_2d_ground_truth).");
DEFINE_double(ground_truth_map_min_cell_width, 0.01,
              "Resolution of the ground truth map in meters.");
DEFINE_bool(
    ground_truth_map_saved_with_floating_precision, true,
    "Whether the ground truth map was saved with floating point precision.");

using namespace wavemap_2d;  // NOLINT
int main(int argc, char** argv) {
  using DataStructureType = DenseGrid<SaturatingOccupancyCell>;
  using GTDataStructureType = DenseGrid<UnboundedScalarCell>;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Load the estimated map
  const auto estimated_map_min_cell_width =
      static_cast<FloatingPoint>(FLAGS_estimated_map_min_cell_width);
  auto estimated_map =
      std::make_shared<DataStructureType>(estimated_map_min_cell_width);
  CHECK(estimated_map->load(FLAGS_estimated_map_file_path,
                            FLAGS_estimated_map_saved_with_floating_precision));

  // Load the ground truth map
  const auto ground_truth_map_min_cell_width =
      static_cast<FloatingPoint>(FLAGS_ground_truth_map_min_cell_width);
  GTDataStructureType ground_truth_map(ground_truth_map_min_cell_width);
  CHECK(ground_truth_map.load(
      FLAGS_ground_truth_map_file_path,
      FLAGS_ground_truth_map_saved_with_floating_precision));

  // Configure evaluations
  utils::MapEvaluationConfig evaluation_config;
  evaluation_config.iterate_over =
      utils::MapEvaluationConfig::Source::kPredicted;
  evaluation_config.crop_to = utils::MapEvaluationConfig::Source::kReference;
  evaluation_config.reference.cell_selector = {
      utils::CellSelector::Categories::kAny};
  evaluation_config.reference.treat_unknown_cells_as =
      OccupancyState::Occupied();
  evaluation_config.predicted.cell_selector = {
      utils::CellSelector::Categories::kAnyObserved};

  // Compress
  constexpr FloatingPoint kThreshold = kEpsilon;
  const Index map_size = estimated_map->dimensions();
  const int max_num_passes = int_math::log2_floor(map_size.minCoeff());

  const auto min_divisor = static_cast<int>(std::exp2(max_num_passes));
  Index new_max_index = estimated_map->getMinIndex();
  if (map_size.x() % min_divisor != 0) {
    const IndexElement new_x_size =
        (map_size.x() / min_divisor + 1) * min_divisor;
    new_max_index.x() += new_x_size - 1;
  }
  if (map_size.y() % min_divisor != 0) {
    const IndexElement new_y_size =
        (map_size.y() / min_divisor + 1) * min_divisor;
    new_max_index.y() += new_y_size - 1;
  }
  estimated_map->addToCellValue(new_max_index, 0.f);

  const size_t num_non_zero_cells_initial =
      (kThreshold < estimated_map->getData().cwiseAbs().array()).count();
  const utils::MapEvaluationSummary estimated_map_evaluation_summary =
      utils::EvaluateMap(ground_truth_map, *estimated_map, evaluation_config);
  CHECK(estimated_map_evaluation_summary.is_valid);
  for (const auto& dwt :
       {std::shared_ptr<DiscreteWaveletTransform<FloatingPoint>>(
            new NaiveHaar<FloatingPoint>()),
        std::shared_ptr<DiscreteWaveletTransform<FloatingPoint>>(
            new LiftedCDF53<FloatingPoint>()),
        std::shared_ptr<DiscreteWaveletTransform<FloatingPoint>>(
            new LiftedCDF97<FloatingPoint>())}) {
    for (int pass_idx = 1; pass_idx < std::min(max_num_passes, 9); ++pass_idx) {
      // Compress
      DataStructureType reconstructed_map(*estimated_map);
      GTDataStructureType error_map(estimated_map->getMinCellWidth());
      dwt->forward(reconstructed_map.getData(), pass_idx);

      // Truncate
      reconstructed_map.getData() =
          (reconstructed_map.getData().cwiseAbs().array() < kThreshold)
              .select(0.f, reconstructed_map.getData());
      const size_t num_non_zero_cells =
          (0 < reconstructed_map.getData().cwiseAbs().array()).count();

      // Decompress
      dwt->backward(reconstructed_map.getData(), pass_idx);

      // Evaluate the accuracy loss
      const utils::MapEvaluationSummary reconstructed_map_evaluation_summary =
          utils::EvaluateMap(ground_truth_map, reconstructed_map,
                             evaluation_config, &error_map);
      CHECK(reconstructed_map_evaluation_summary.is_valid);

      // Print the comparison
      std::ostringstream summary_comparison;
      summary_comparison
          << "Total non-zero cells at pass " << pass_idx << ": "
          << num_non_zero_cells << "  ("
          << (static_cast<float>(num_non_zero_cells) /
              static_cast<float>(num_non_zero_cells_initial) * 100.f)
          << "%)\n"
          << "Evaluated "
          << reconstructed_map_evaluation_summary.num_cells_evaluated()
          << " vs " << estimated_map_evaluation_summary.num_cells_evaluated()
          << ", of which "
          << reconstructed_map_evaluation_summary.num_cells_considered()
          << " vs " << estimated_map_evaluation_summary.num_cells_considered()
          << " were considered and "
          << reconstructed_map_evaluation_summary.num_cells_ignored << " vs "
          << estimated_map_evaluation_summary.num_cells_ignored << " ignored.\n"
          << "Precision: " << reconstructed_map_evaluation_summary.precision()
          << " vs " << estimated_map_evaluation_summary.precision() << "\n"
          << "Recall: " << reconstructed_map_evaluation_summary.recall()
          << " vs " << estimated_map_evaluation_summary.recall() << "\n"
          << "F1 score: " << reconstructed_map_evaluation_summary.f_1_score()
          << " vs " << estimated_map_evaluation_summary.f_1_score() << "\n";
      LOG(INFO) << summary_comparison.str();
    }
    std::cout << std::endl;
  }

  // Evaluate compression in a quadtree
  SimpleQuadtree<UnboundedOccupancyCell> quadtree(estimated_map_min_cell_width);
  for (const Index& index :
       Grid(estimated_map->getMinIndex(), estimated_map->getMaxIndex())) {
    quadtree.setCellValue(index, estimated_map->getCellValue(index));
  }
  LOG(INFO) << "Dense grid memory usage: "
            << estimated_map->getMemoryUsage() / 1000 << " KB.";
  LOG(INFO) << "Size: " << estimated_map->size();
  LOG(INFO) << "Quadtree memory usage: " << quadtree.getMemoryUsage() / 1000
            << " KB.";
  LOG(INFO) << "Size: " << quadtree.size();
  quadtree.prune();
  LOG(INFO) << "Quadtree memory usage: " << quadtree.getMemoryUsage() / 1000
            << " KB.";
  for (auto& node :
       quadtree.getNodeIterator<TraversalOrder::kDepthFirstPostorder>()) {
    AverageAndPruneChildren<UnboundedOccupancyCell::Specialized>(node);
  }
  LOG(INFO) << "Quadtree memory usage: " << quadtree.getMemoryUsage() / 1000
            << " KB.";
  LOG(INFO) << "Size: " << quadtree.size();
}
