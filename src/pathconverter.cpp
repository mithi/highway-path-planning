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

    xs.push_back(x);
    ys.push_back(y);
    ss.push_back(s);
    dxs.push_back(dx);
    dys.push_back(dy);

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

  xs.push_back(x1);
  ys.push_back(y1);
  ss.push_back(distance);
  dxs.push_back(dx1);
  dys.push_back(dy1);

  // Uses the loaded information to fit cubic polynomial curves
  this->x_spline.set_points(ss, xs);
  this->y_spline.set_points(ss, ys);
  this->dx_spline.set_points(ss, dxs);
  this->dy_spline.set_points(ss, dys);
  this->distance = distance;
}

void PathConverter::save(std::string file_path, const double t, const int n){

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

void PathConverter::save(std::string file_path, const double t, const int n, const double d){

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

vector<double> PathConverter::convert_sd_to_xy(const double s, const double d) {

  const double x_edge = this->x_spline(s);
  const double y_edge = this->y_spline(s);
  const double dx = this->dx_spline(s);
  const double dy = this->dy_spline(s);

  const double x = x_edge + dx * d;
  const double y = y_edge + dy * d;

  return {x, y};
}
