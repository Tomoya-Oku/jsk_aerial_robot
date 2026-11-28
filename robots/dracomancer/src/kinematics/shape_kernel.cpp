// Compute DRAGON centerline samples for a batch of joint angles.
#include <Python.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <vector>

namespace
{
constexpr int kJointCount = 6;
constexpr int kJointWidth = 16;  // Row-major rotation, translation, axis, angle index.
constexpr double kMinSegment = 1e-10;
using Vector = Eigen::Vector3d;
using Matrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
using Points = std::vector<Vector>;

class Array
{
public:
  Array() = default;
  Array(const Array&) = delete;
  ~Array()
  {
    if (view_.obj)
      PyBuffer_Release(&view_);
  }

  bool read(PyObject* object, int dimensions, bool writable = false)
  {
    const int flags = PyBUF_C_CONTIGUOUS | PyBUF_FORMAT | (writable ? PyBUF_WRITABLE : 0);
    if (PyObject_GetBuffer(object, &view_, flags) < 0)
      return false;
    if (view_.ndim != dimensions || view_.itemsize != sizeof(double) || !view_.format ||
        std::strcmp(view_.format, "d") != 0)
    {
      PyErr_SetString(PyExc_ValueError, "expected a contiguous native float64 array");
      return false;
    }
    return true;
  }

  Py_ssize_t size(int axis) const { return view_.shape[axis]; }
  double* data() const { return static_cast<double*>(view_.buf); }

private:
  Py_buffer view_{};
};

struct Joint
{
  Eigen::Isometry3d origin;
  Vector axis;
  int index;
};

std::vector<Joint> readChain(const Array& packed)
{
  std::vector<Joint> chain;
  chain.reserve(packed.size(0));
  for (Py_ssize_t i = 0; i < packed.size(0); ++i)
  {
    const auto* row = packed.data() + i * kJointWidth;
    const Eigen::Map<const Eigen::Matrix<double, kJointWidth, 1>> fields(row);
    if (!fields.allFinite() || row[15] < -1 || row[15] >= kJointCount || std::floor(row[15]) != row[15])
      throw std::invalid_argument("invalid packed URDF joint");
    Joint joint{Eigen::Isometry3d::Identity(), Eigen::Map<const Vector>(row + 12), static_cast<int>(row[15])};
    joint.origin.linear() = Eigen::Map<const Matrix>(row);
    joint.origin.translation() = Eigen::Map<const Vector>(row + 9);
    if (joint.index >= 0 && std::abs(joint.axis.squaredNorm() - 1.) > 1e-8)
      throw std::invalid_argument("invalid packed URDF axis");
    chain.push_back(joint);
  }
  return chain;
}

Eigen::Isometry3d forward(const std::vector<Joint>& chain, const double* angles, Points* points = nullptr)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  for (const Joint& joint : chain)
  {
    pose = pose * joint.origin;
    if (points && (pose.translation() - points->back()).norm() > kMinSegment)
      points->push_back(pose.translation());
    if (joint.index >= 0)
      pose.rotate(Eigen::AngleAxisd(angles[joint.index], joint.axis));
  }
  return pose;
}

void sample(const Points& points, const Eigen::Matrix3d& rotation, const Vector& reference,
            Py_ssize_t samples, double* output, std::vector<double>& arc)
{
  arc.resize(points.size());
  arc[0] = 0.;
  for (std::size_t i = 1; i < points.size(); ++i)
  {
    const double length = (points[i] - points[i - 1]).norm();
    if (!(length > kMinSegment) || !std::isfinite(length))
      throw std::invalid_argument("zero-length or invalid DRAGON segment");
    arc[i] = arc[i - 1] + length;
  }
  const double total = arc.back();
  std::size_t segment = 1;
  for (Py_ssize_t i = 0; i < samples; ++i)
  {
    const double distance = i == samples - 1 ? total : total * static_cast<double>(i) / (samples - 1);
    while (segment + 1 < points.size() && distance > arc[segment])
      ++segment;
    const double weight = (distance - arc[segment - 1]) / (arc[segment] - arc[segment - 1]);
    const Vector point = points[segment - 1] + weight * (points[segment] - points[segment - 1]);
    Eigen::Map<Vector> result(output + i * 3);
    result = rotation * (point - reference) / total;
  }
}

PyObject* sampleCenterlines(PyObject*, PyObject* args)
{
  PyObject *angle_obj, *chain_obj, *fc_obj, *mid_obj, *tail_obj, *align_obj, *output_obj;
  int centered;
  if (!PyArg_ParseTuple(args, "OOOOOOOp", &angle_obj, &chain_obj, &fc_obj, &mid_obj,
                        &tail_obj, &align_obj, &output_obj, &centered))
    return nullptr;
  Array angles, chain_array, fc_array, mid_array, tail, align, output;
  if (!angles.read(angle_obj, 2) || !chain_array.read(chain_obj, 2) ||
      !fc_array.read(fc_obj, 2) || !mid_array.read(mid_obj, 2) ||
      !tail.read(tail_obj, 1) || !align.read(align_obj, 2) || !output.read(output_obj, 3, true))
    return nullptr;
  if (angles.size(1) != kJointCount || chain_array.size(1) != kJointWidth ||
      fc_array.size(1) != kJointWidth || mid_array.size(1) != kJointWidth ||
      tail.size(0) != 3 || align.size(0) != 3 || align.size(1) != 3 ||
      output.size(0) != angles.size(0) || output.size(1) < 2 || output.size(2) != 3)
  {
    PyErr_SetString(PyExc_ValueError, "invalid DRAGON centerline array dimensions");
    return nullptr;
  }

  try
  {
    const auto chain = readChain(chain_array);
    const auto fc_chain = readChain(fc_array);
    const auto mid_chain = readChain(mid_array);
    const Eigen::Map<const Vector> tail_point(tail.data());
    const Eigen::Map<const Matrix> alignment(align.data());
    if (!tail_point.allFinite() || !alignment.allFinite())
      throw std::invalid_argument("invalid DRAGON centerline alignment");
    Points points;
    std::vector<double> arc;
    points.reserve(chain.size() + 2);
    arc.reserve(chain.size() + 2);

    for (Py_ssize_t i = 0; i < angles.size(0); ++i)
    {
      const double* q = angles.data() + i * kJointCount;
      const Eigen::Map<const Eigen::Matrix<double, kJointCount, 1>> joint_angles(q);
      if (!joint_angles.allFinite())
        throw std::invalid_argument("invalid DRAGON joint angles");
      points.assign(1, Vector::Zero());
      const auto end = forward(chain, q, &points);
      const Vector tip = end * tail_point;
      points.push_back(tip);
      std::reverse(points.begin(), points.end());

      // Work in link1 coordinates. Composing through the common root cancels
      // it when expressing the points in FC axes about the middle joint.
      Eigen::Matrix3d frame = end.linear();
      Vector reference = tip;
      if (centered)
      {
        frame = forward(fc_chain, q).linear();
        reference = forward(mid_chain, q).translation();
      }
      sample(points, alignment * frame.transpose(), reference, output.size(1),
             output.data() + i * output.size(1) * 3, arc);
    }
  }
  catch (const std::exception& error)
  {
    PyErr_SetString(PyExc_ValueError, error.what());
    return nullptr;
  }
  Py_RETURN_NONE;
}

PyMethodDef methods[] = {
  {"sample_centerlines", sampleCenterlines, METH_VARARGS, "Fill the output with sampled DRAGON centerlines."},
  {nullptr, nullptr, 0, nullptr},
};
PyModuleDef module = {
  PyModuleDef_HEAD_INIT, "_shape_kernel", "DRAGON kinematics and centerline sampling.", -1, methods,
  nullptr, nullptr, nullptr, nullptr,
};
}  // namespace

PyMODINIT_FUNC PyInit__shape_kernel()
{
  return PyModule_Create(&module);
}
