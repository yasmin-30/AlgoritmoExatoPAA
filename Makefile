CXX ?= g++
CXXFLAGS ?= -std=c++17 -O3
CPPFLAGS := -Iinclude -Iinclude/heuristica

ifeq ($(OS),Windows_NT)
TARGET := mTSP_DS.exe
RM := del /Q
else
TARGET := mTSP_DS
RM := rm -f
endif

SOURCES := \
	src/main.cpp \
	src/leitorArquivos.cpp \
	src/salvarExecExata.cpp \
	src/exato/algExato.cpp \
	src/exato/ramificacoes.cpp \
	src/exato/custos.cpp \
	src/heuristica/heuristica.cpp \
	src/heuristica/execUnica.cpp

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(SOURCES) -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)
