#include <iostream>
#include <vector>
#include <functional>

struct RoutineInfo {
    const std::string name;
    const std::string desc;
};

struct Routine : RoutineInfo {
    std::function<void()> actions;
};


#define ROUTINE_DEFINITIONS_FOR(robot) \
void robot::initializeRoutines()
#define START_ROUTINE(name, desc) \
        routines.push_back( \
            {name, \
            desc, \
            [this]() {
#define END_ROUTINE \
            }});
 
class Robot {
public:
    Robot() = delete;
    
    template <typename SpecRobot>
    Robot(SpecRobot* spec) {
        spec->initializeRoutines(); 
    }

    virtual void autonomous() {
        // This will later be received from the RoutineSelector.
        const std::size_t routineIndex{0}; 
        routines[routineIndex].actions();
    }

protected:
    std::vector<Routine> routines;
};

class Robot15 : public Robot {
public:
  friend class Robot;
    Robot15(const int iSpecialNumber) : Robot{this}, specialNumber{iSpecialNumber} {}
private:
    void initializeRoutines();
    const int specialNumber;
};

ROUTINE_DEFINITIONS_FOR(Robot15) {
    START_ROUTINE("This is a test.", "A test that this may work and nothing more")
        std::cout << "Well, how about that?\n";
        std::cout << specialNumber << '\n';
    END_ROUTINE
}

class Robot24 : public Robot {
public:
  friend class Robot;
  Robot24(const std::string &iSpecialWord) : Robot{this}, specialWord{iSpecialWord} {}
    
private:
    void initializeRoutines();
    const std::string specialWord;
};

ROUTINE_DEFINITIONS_FOR(Robot24) {
    START_ROUTINE("This is a test.", "A test that this may work and nothing more")
        std::cout << "Well, how about that?\n";
        std::cout << specialWord << '\n';
    END_ROUTINE
}

int main() {
    Robot15 test15{2};
    test15.autonomous();
    Robot24 test24{"dang"};
    test24.autonomous();
    return 0;
}