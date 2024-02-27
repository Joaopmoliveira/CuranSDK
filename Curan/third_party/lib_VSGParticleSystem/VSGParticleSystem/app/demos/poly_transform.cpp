#include <iostream>
#include <memory>
#include <vector>

class Base {
public:
  virtual void commonMethod() const { std::cout << "Base common method\n"; }
};

class A : public Base {
public:
  void commonMethod() const override { std::cout << "A common method\n"; }
};

class B : public Base {
public:
  void commonMethod() const override { std::cout << "B common method\n"; }
};

class Pipeline {
public:
  // void add(std::unique_ptr<Base> element) {
  //     elements.push_back(std::move(element));
  // }
  template <typename T> void operator<<(T const &b) {
    std::unique_ptr<Base> t = std::make_unique<T>(b);
    elements.push_back(std::move(t));
  }

  void operator()() const {
    for (const auto &element : elements) {
      element->commonMethod();
    }
  }

private:
  std::vector<std::unique_ptr<Base>> elements;
};

class Teste {
public:
  Teste() {}

  template <typename T> void operator<<(std::pair<std::string, T> const &a) {
    std::cout << a.first << " " << a.second << "\n";
  }
};

int main() {
  A a1 = A();
  A a2 = A();
  B b1 = B();
  B b2 = B();

  Pipeline pipeline;
  pipeline << a1;
  pipeline << a2;
  pipeline << b1;
  pipeline << b2;

  pipeline();

  return 0;
}
