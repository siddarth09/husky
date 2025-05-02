// RF Leg IK Solver (analytical, 3 DoF)
#include <cmath>
#include <vector>
#include <optional>
#include <stdexcept>

struct LegIKResult {
  double haa;  // Hip abduction
  double hfe;  // Hip flexion/extension
  double kfe;  // Knee flexion/extension
};

enum class IKMode {
  ELBOW_UP,
  ELBOW_DOWN
};

std::optional<LegIKResult> solve_leg_ik_rf(
  double x, double y, double z,
  IKMode mode = IKMode::ELBOW_DOWN
) {
  // Link lengths (you MUST replace these with actual values from your URDF)
  const double L1 = 0.05; // hip offset (RF_HAA frame to RF_HFE frame in y)
  const double L2 = 0.2;  // thigh length (RF_HFE to RF_KFE)
  const double L3 = 0.2;  // shin length (RF_KFE to foot)

  // Step 1: Compute hip abduction (rotation around z-axis)
  double d = std::sqrt(y*y + z*z - L1*L1);
  if (d < 1e-6) return std::nullopt;  // Singularity at center
  double theta_haa = std::atan2(y, z) - std::asin(L1 / std::sqrt(y*y + z*z));

  // Step 2: Rotate foot into sagittal (x-z) plane of leg
  double y_proj = y - L1 * std::sin(theta_haa);
  double z_proj = z - L1 * std::cos(theta_haa);

  // Step 3: Planar 2D IK
  double D = (x*x + z_proj*z_proj - L2*L2 - L3*L3) / (2 * L2 * L3);

  if (std::abs(D) > 1.0) return std::nullopt; // No solution

  double theta_kfe = (mode == IKMode::ELBOW_DOWN) ? std::atan2(-std::sqrt(1 - D*D), D)
                                                  : std::atan2(+std::sqrt(1 - D*D), D);

  double phi = std::atan2(z_proj, x);
  double psi = std::atan2(L3 * std::sin(theta_kfe), L2 + L3 * std::cos(theta_kfe));

  double theta_hfe = phi - psi;

  return LegIKResult{theta_haa, theta_hfe, theta_kfe};
}

auto ik_result = solve_leg_ik_rf(0.25, 0.0, -0.3, IKMode::ELBOW_DOWN);
if (ik_result) {
  std::cout << "HAA: " << ik_result->haa << "\n";
  std::cout << "HFE: " << ik_result->hfe << "\n";
  std::cout << "KFE: " << ik_result->kfe << "\n";
} else {
  std::cerr << "IK failed: position unreachable or singular.\n";
}

