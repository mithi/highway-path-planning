#include "pathconverter.h"

using namespace std;

PathConverter::PathConverter(std::string file_path, const double distance) {

  vector<double> xs;
  vector<double> ys;
  vector<double> ss;
  vector<double> dxs;
  vector<double> dys;

  double x, y, s, dx, dy;
  double x1, y1, dx1, dy1;

  bool first = true;

  string line;
  ifstream in_file(file_path.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cerr << "PathConverter::PathConverter - Cannot open input file: " << file_path << endl;
    exit(EXIT_FAILURE);
  }

  // Load information from file to memory
  while (getline(in_file, line)) {

    istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    xs.emplace_back(x);
    ys.emplace_back(y);
    ss.emplace_back(s);
    dxs.emplace_back(dx);
    dys.emplace_back(dy);

    if (first) {
      x1 = x;
      y1 = y;
      dx1 = dx;
      dy1 = dy;
      first = false;
    }
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  xs.emplace_back(x1);
  ys.emplace_back(y1);
  ss.emplace_back(distance);
  dxs.emplace_back(dx1);
  dys.emplace_back(dy1);

  // Uses the loaded information to fit cubic polynomial curves
  this->x_spline.set_points(ss, xs);
  this->y_spline.set_points(ss, ys);
  this->dx_spline.set_points(ss, dxs);
  this->dy_spline.set_points(ss, dys);
  this->distance = distance;
}

vector<double> PathConverter::convert_sd_to_xy(const double s, const double d) const {

  const double mod_s = fmod(s, this->distance);
  const double x_edge = this->x_spline(mod_s);
  const double y_edge = this->y_spline(mod_s);
  const double dx = this->dx_spline(mod_s);
  const double dy = this->dy_spline(mod_s);

  const double x = x_edge + dx * d;
  const double y = y_edge + dy * d;

  return {x, y};
}

XYPoints PathConverter::make_path(JMT jmt_s, JMT jmt_d, const double t, const int n) const {

  vector<double> xs;
  vector<double> ys;
  vector<double> p;

  for (int i = 0; i < n; i++) {

    double s = jmt_s.get(i * t);
    double d = jmt_d.get(i * t);

    vector<double> p = this->convert_sd_to_xy(s, d);

    xs.emplace_back(p[0]);
    ys.emplace_back(p[1]);
  }

  XYPoints path = {xs, ys, n};

  return path;
}

void PathConverter::save(std::string file_path, const double t, const int n) const {

  ofstream out_file(file_path.c_str(), ofstream::out);

  if (!out_file.is_open()) {
    cerr << "PathConverter::save() - Cannot open output file: " << file_path << endl;
    exit(EXIT_FAILURE);
  }

  for (int i = 0; i < n; i++) {

    const double s = i * t;

    out_file << this->x_spline(s) << " ";
    out_file << this->y_spline(s) << " ";
    out_file << s << " ";
    out_file << this->dx_spline(s) << " ";
    out_file << this->dy_spline(s) << "\n";
  }

  if (out_file.is_open()) {
    out_file.close();
  }
}

void PathConverter::save(std::string file_path, const double t, const int n, const double d) const {

  ofstream out_file(file_path.c_str(), ofstream::out);

  if (!out_file.is_open()) {
    cerr << "PathConverter::save() - Cannot open output file: " << file_path << endl;
    exit(EXIT_FAILURE);
  }

  for (int i = 0; i < n; i++) {

    const double s = i * t;
    vector<double> p = convert_sd_to_xy(s, d);

    out_file << p[0] << " ";
    out_file << p[1] << " ";
    out_file << s << " ";
    out_file << this->dx_spline(s) << " ";
    out_file << this->dy_spline(s) << "\n";
  }

  if (out_file.is_open()) {
    out_file.close();
  }
}
