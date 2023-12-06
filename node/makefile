SRC_FOLDER:=src
OUT_FOLDER:=compile
TEST_FOLDER:=test

SRC:=$(wildcard $(SRC_FOLDER)/*.cpp)
SRC_OBJ:=$(SRC:$(SRC_FOLDER)/%=$(OUT_FOLDER)/%.o)
SRC_COMP_OBJ:=$(SRC:$(SRC_FOLDER)/%=$(OUT_FOLDER)/%.comp)

TEST_SRC:=$(wildcard $(TEST_FOLDER)/*.cpp)
TEST_OBJ:=$(TEST_SRC:$(TEST_FOLDER)/%=$(OUT_FOLDER)/%.test.o)
TEST_TOTAL_OBJ:=$(SRC:$(SRC_FOLDER)/%=$(OUT_FOLDER)/%.test.o)
TEST_TOTAL_OBJ+=$(TEST_OBJ)

TEST_OBJ_COMP:=$(TEST_SRC:$(TEST_FOLDER)/%.cpp=$(OUT_FOLDER)/%.cpp.comp)
SRC_COMP_OBJ+=$(TEST_OBJ_COMP)

CXX:=g++
CXX_INCLUDE:=-I./include -I/opt/ros/noetic/include
CXX_FLAGS:=-Wall -Wextra
CXX_LINK_FLAGS:=-L/opt/ros/noetic/lib -L/main/bin/ -lpthread -lroscpp -lrosconsole -lroscpp_serialization -lrostime

OUT_FILE:=node

COMPILE_COMMAND_EXEC:=comp_database_gen
COMPILE_COMMANDS_FILE:=compile_commands.json
PWD:=$(shell pwd)

define COMPILE =
$(CXX) $(CXX_FLAGS) $(CXX_INCLUDE) -c $< -o $@
endef

# MAIN BUILD
$(OUT_FOLDER)/%.o:$(SRC_FOLDER)/%
	$(COMPILE)

$(OUT_FILE):$(SRC_OBJ)
	$(CXX) $(CXX_FLAGS) $^ -o $@ $(CXX_LINK_FLAGS)

# BUILD TESTS
$(OUT_FOLDER)/%.test.o:$(SRC_FOLDER)/%
	$(COMPILE) -DTEST_EXEC
$(OUT_FOLDER)/%.test.o:$(TEST_FOLDER)/%
	$(COMPILE) -DTEST_EXEC
tests: $(TEST_TOTAL_OBJ)
	$(CXX) $(CXX_FLAGS) $^ -o $@ $(CXX_LINK_FLAGS) -lgtest

# COMPILE COMMANDS
$(OUT_FOLDER)/%.comp:$(SRC_FOLDER)/%
	$(shell $(COMPILE_COMMAND_EXEC) $(COMPILE_COMMANDS_FILE) $(PWD) "$(COMPILE)" $<)
$(OUT_FOLDER)/%.comp:$(TEST_FOLDER)/%
	$(shell $(COMPILE_COMMAND_EXEC) $(COMPILE_COMMANDS_FILE) $(PWD) "$(COMPILE) -DTEST_EXEC" $<)
commands: $(SRC_COMP_OBJ)
	$(shell $(COMPILE_COMMAND_EXEC) $(COMPILE_COMMANDS_FILE) --end)

start: $(OUT_FILE)
	./$(OUT_FILE) 50101

clean:
	rm -rf $(OUT_FOLDER)/*
	rm -f $(OUT_FILE)
	rm -f tests