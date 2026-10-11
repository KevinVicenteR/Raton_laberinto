#include "maze_io.h"

#include <fstream>
#include <istream>
#include <ostream>
#include <random>
#include <vector>

using raton::Cell;
using raton::Dir;
using raton::Maze;

namespace sim {

namespace {

char charAt(const std::string& line, size_t i) { return i < line.size() ? line[i] : ' '; }

Cell cell(int x, int y) { return Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)}; }

}  // namespace

bool parseMaze(std::istream& in, Maze& out, std::string& error) {
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    // Ignora lineas vacias al inicio y al final
    if (line.find_first_not_of(" \t") == std::string::npos) {
      if (!lines.empty()) break;
      continue;
    }
    lines.push_back(line);
  }

  if (lines.size() < 3 || lines.size() % 2 == 0) {
    error = "numero de lineas invalido (" + std::to_string(lines.size()) + ")";
    return false;
  }
  const size_t lastPost = lines[0].find_last_not_of(" \t");
  if (lastPost == std::string::npos || lastPost % 4 != 0 || lastPost < 4) {
    error = "la primera linea debe ser de postes: o---o---o";
    return false;
  }
  const int width = static_cast<int>(lastPost / 4);
  const int height = static_cast<int>((lines.size() - 1) / 2);
  if (width > raton::MAX_MAZE_SIZE || height > raton::MAX_MAZE_SIZE) {
    error = "laberinto mayor de 16x16";
    return false;
  }

  out.reset(width, height);
  // Todas las paredes interiores empiezan conocidas y abiertas.
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      if (x + 1 < width) out.setWall(cell(x, y), raton::EAST, false);
      if (y + 1 < height) out.setWall(cell(x, y), raton::NORTH, false);
    }
  }

  for (size_t i = 0; i < lines.size(); ++i) {
    const std::string& l = lines[i];
    if (i % 2 == 0) {
      // Linea de postes: paredes horizontales. La linea i es el borde norte de la fila height-1-i/2.
      const int yBelow = height - 1 - static_cast<int>(i / 2);
      for (int x = 0; x < width; ++x) {
        const char c = charAt(l, static_cast<size_t>(4 * x + 2));
        if (c == ' ') continue;
        if (yBelow >= 0) {
          out.setWall(cell(x, yBelow), raton::NORTH, true);
        } else {
          out.setWall(cell(x, 0), raton::SOUTH, true);
        }
      }
    } else {
      // Linea de celdas: paredes verticales.
      const int y = height - 1 - static_cast<int>(i / 2);
      for (int x = 0; x <= width; ++x) {
        const char c = charAt(l, static_cast<size_t>(4 * x));
        if (c == ' ') continue;
        if (x < width) {
          out.setWall(cell(x, y), raton::WEST, true);
        } else {
          out.setWall(cell(width - 1, y), raton::EAST, true);
        }
      }
    }
  }
  return true;
}

bool loadMaze(const std::string& path, Maze& out, std::string& error) {
  std::ifstream file(path);
  if (!file) {
    error = "no se pudo abrir " + path;
    return false;
  }
  return parseMaze(file, out, error);
}

void writeMaze(std::ostream& out, const Maze& maze) {
  const int w = maze.width();
  const int h = maze.height();
  for (int y = h - 1; y >= -1; --y) {
    // Borde norte de la fila y (o borde sur de la fila 0 cuando y == -1)
    out << 'o';
    for (int x = 0; x < w; ++x) {
      const bool wall = y >= 0 ? maze.hasWall(cell(x, y), raton::NORTH) : maze.hasWall(cell(x, 0), raton::SOUTH);
      out << (wall ? "---" : "   ") << 'o';
    }
    out << '\n';
    if (y < 0) break;
    for (int x = 0; x < w; ++x) {
      out << (maze.hasWall(cell(x, y), raton::WEST) ? '|' : ' ') << "   ";
    }
    out << (maze.hasWall(cell(w - 1, y), raton::EAST) ? '|' : ' ') << '\n';
  }
}

Maze generateMaze(int width, int height, uint32_t seed, int extraOpenings) {
  Maze maze(width, height);
  width = maze.width();
  height = maze.height();
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      for (int d = 0; d < 4; ++d) maze.setWall(cell(x, y), static_cast<Dir>(d), true);
    }
  }

  std::mt19937 rng(seed);
  std::vector<std::vector<bool>> seen(width, std::vector<bool>(height, false));
  std::vector<Cell> stack;

  // La salida queda fuera del backtracker y solo se abre al norte.
  seen[0][0] = true;
  const Cell first = height > 1 ? cell(0, 1) : cell(1, 0);
  seen[first.x][first.y] = true;
  stack.push_back(first);
  maze.setWall(cell(0, 0), height > 1 ? raton::NORTH : raton::EAST, false);

  while (!stack.empty()) {
    const Cell c = stack.back();
    Dir options[4];
    int n = 0;
    for (int d = 0; d < 4; ++d) {
      const Cell nb = raton::step(c, static_cast<Dir>(d));
      if (maze.inside(nb) && !seen[nb.x][nb.y]) options[n++] = static_cast<Dir>(d);
    }
    if (n == 0) {
      stack.pop_back();
      continue;
    }
    const Dir d = options[rng() % n];
    const Cell nb = raton::step(c, d);
    maze.setWall(c, d, false);
    seen[nb.x][nb.y] = true;
    stack.push_back(nb);
  }

  // La meta del centro es una sola sala abierta.
  Cell goals[4];
  const int g = maze.goalCells(goals);
  for (int i = 0; i < g; ++i) {
    for (int j = 0; j < g; ++j) {
      for (int d = 0; d < 4; ++d) {
        if (raton::step(goals[i], static_cast<Dir>(d)) == goals[j]) maze.setWall(goals[i], static_cast<Dir>(d), false);
      }
    }
  }

  // Aberturas extra para que haya mas de un camino.
  for (int k = 0, tries = 0; k < extraOpenings && tries < extraOpenings * 20; ++tries) {
    const Cell c = cell(static_cast<int>(rng() % width), static_cast<int>(rng() % height));
    const Dir d = (rng() & 1) ? raton::NORTH : raton::EAST;
    const Cell nb = raton::step(c, d);
    if (!maze.inside(nb) || c == cell(0, 0) || nb == cell(0, 0) || !maze.hasWall(c, d)) continue;
    maze.setWall(c, d, false);
    k++;
  }
  return maze;
}

}  // namespace sim
