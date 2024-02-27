#include <vsgParticleSystem/transformation/Transformation.h>

#include <iostream>

namespace vsgps {
Transformation::Transformation(size_t transformationIdx)
    : idx{transformationIdx} {}

void Transformation::UnimplementedMethod() {
    throw std::runtime_error("Unimplemented Transformation's method! You tried "
                             "to use a method that wasn't yet implemented.");
}

void Transformation::operator()(State &xi) { UnimplementedMethod(); }

void Transformation::operator()(State const &xi, State &xitilde) {
    UnimplementedMethod();
}

void Transformation::inv(State &xitilde) { UnimplementedMethod(); }

void Transformation::inv(State const &xitilde, State &xi) {
    UnimplementedMethod();
}

void Transformation::invVel(State &dxitilde) {
    this->inv(dxitilde);
}

void Transformation::invVel(State const &dxitilde, State &dxi) {
    this->inv(dxitilde, dxi);
}

Transformation Transformation::fromJson(std::string path) {
    return Transformation(0);
}
void Transformation::setIndex(size_t newIdx) { idx = newIdx; }

size_t const &Transformation::getIndex() const { return idx; }

std::ostream &operator<<(std::ostream &out, Transformation const &t) {
    return out << "Transformation{\n\tid: " << t.idx << ",\n};\n";
}

} // namespace vsgps
