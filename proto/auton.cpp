#include <iostream>
#include <memory>

class Routine;

class Robot {
  public:
  virtual void autonomous(Routine *routine) = 0;
};

class Robot15In;
class Robot24In;
struct Routine {
  virtual void go(Robot15In *bot) = 0;
  virtual void go(Robot24In *bot) = 0;
  const std::string start;
  const std::string desc;
};

class Robot15In : public Robot {
  public:
  void autonomous(Routine *routine) override {
    routine->go(this);
  }
  const std::string secret15{"I am 15\" mane."};
};

class Robot24In : public Robot {
  public:
  void autonomous(Routine *routine) override {
    routine->go(this);
  }
  const std::string secret24{"I am 24\" mane."};
};

struct TestRoutine : Routine {
  void go(Robot15In *bot) override {
    std::cout << bot->secret15 << '\n';
  }

  void go(Robot24In *bot) override {
    std::cout << bot->secret24 << '\n';
  }

  const std::string start{"Begin at (0, 0, 0) with preload beside you."};
  const std::string desc{"Does nothing, just for testing."};
};

int main() {
  std::unique_ptr<Robot> robot = std::make_unique<Robot24In>();
  auto routine = std::make_unique<TestRoutine>();
  std::cout << routine->start << '\n';
  robot->autonomous(routine.get());
}