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
    Robot(Robot* spec) {
        spec->initializeRoutines();
    }

    virtual void autonomous() {
        // This will later be received from the RoutineSelector.
        const std::size_t routineIndex{0}; 
        routines[routineIndex].actions();
    }

protected:
    virtual void initializeRoutines() = 0;
    std::vector<Routine> routines;
};


class Robot15 : public Robot<Robot15> {
public:
    Robot15(const int iSpecialNumber) : specialNumber{iSpecialNumber} {}
    
private:
    void initializeRoutines() override;
    const int specialNumber;
};

ROUTINE_DEFINITIONS_FOR(Robot15) {
    START_ROUTINE("This is a test.", "A test that this may work and nothing more")
        std::cout << "Well, how about that?\n";
        std::cout << specialNumber << '\n';
    END_ROUTINE
}


class Robot24 : public Robot<Robot24> {
public:
    Robot24(const std::string &iSpecialWord) : specialWord{iSpecialWord} {}
    
private:
    void initializeRoutines() override;
    const std::string specialWord;
};

ROUTINE_DEFINITIONS_FOR(Robot24) {
    START_ROUTINE("This is a test.", "A test that this may work and nothing more")
        std::cout << "Well, how about that?\n";
        std::cout << specialWord << '\n';
    END_ROUTINE
}
    

int main() {
    Robot15 test{2};
    test.autonomous();
    return 0;
}