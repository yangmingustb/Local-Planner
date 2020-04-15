
#include "selfType.h"

namespace lattice_planner {

Node::Node(double s, double rho, double cost, int id, int pid) {
  this->s_ = s;
  this->rho_ = rho;
  this->cost_ = cost;
  this->id_ = id;
  this->pid_ = pid;
}

void Node::PrintStatus() {
  std::cout << "--------------" << std::endl
            << "Node          :" << std::endl
            << "s             : " << s_ << std::endl
            << "rho             : " << rho_ << std::endl
            << "Cost          : " << cost_ << std::endl
            << "Id            : " << id_ << std::endl
            << "Pid           : " << pid_ << std::endl
            << "--------------" << std::endl;
}

Node Node::operator+(Node p) {
  Node tmp;
  tmp.s_ = this->s_ + p.s_;
  tmp.rho_ = p.rho_;
  tmp.cost_ = this->cost_ + p.cost_;
  return tmp;
}

Node Node::operator-(Node p) {
  Node tmp;
  tmp.s_ = this->s_ - p.s_;
  tmp.rho_ = this->rho_ - p.rho_;
  return tmp;
}

void Node::operator=(Node p) {
  this->s_ = p.s_;
  this->rho_ = p.rho_;
  this->cost_ = p.cost_;
  this->id_ = p.id_;
  this->pid_ = p.pid_;
}

bool Node::operator==(Node p) {
  if (this->s_ == p.s_ && this->rho_ == p.rho_) return true;
  return false;
}

bool Node::operator!=(Node p) {
  if (this->s_ != p.s_ || this->rho_ != p.rho_) return true;
  return false;
}

bool compare_cost::operator()(Node& p1, Node& p2) {
  if (p1.cost_ >= p2.cost_) return true;
  return false;
};

}  // namespace name
