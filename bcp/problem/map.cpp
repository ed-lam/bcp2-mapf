/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "problem/map.h"
#include "types/string.h"
#include <fstream>

Map::Map(const FilePath& map_path)
{
    // Open map file.
    std::ifstream map_file;
    map_file.open(map_path, std::ios::in);
    ASSERT(map_file.good(), "Invalid map file {}", map_path.string());

    // Read map.
    {
        char buf[1024];
        map_file.getline(buf, 1024);
        ASSERT(strstr(buf, "type octile"), "Invalid map file format");
    }

    // Read map size.
    Position width = 0;
    Position height = 0;
    String param;
    {
        Position value;
        for (Size64 i = 0; i < 2; ++i)
        {
            map_file >> param >> value;
            if (param == "width")
            {
                width = value;
            }
            else if (param == "height")
            {
                height = value;
            }
            else
            {
                ERROR("Invalid input in map file {}", param);
            }
        }
        ASSERT(height > 0, "Invalid map height {}", height);
        ASSERT(width > 0, "Invalid map width {}", width);
        height += 2; // Add border.
        width += 2;
    }

    // Create map.
    resize(width, height);

    // Read grid.
    map_file >> param;
    ASSERT(param == "map", "Invalid map file header");
    {
        auto c = static_cast<char>(map_file.get());
        ASSERT(map_file.good(), "Invalid map header");
        if (c == '\r')
        {
            c = static_cast<char>(map_file.get());
            ASSERT(map_file.good(), "Invalid map header");
        }
        ASSERT(c == '\n', "Invalid map header");
    }
    Node n = width + 1; // Start reading into the second row, second column of the grid
    while (true)
    {
        auto c = static_cast<char>(map_file.get());
        if (!map_file.good())
        {
            // eof
            break;
        }

        switch (c)
        {
            case '\n':
                if (auto c2 = map_file.peek(); map_file.good() && c2 != '\r' && c2 != '\n')
                {
                    n += 2;
                }
                break;
            case ' ':
            case '\t':
            case '\r':
                continue;
            case '.':
                ASSERT(n < size(), "More cells in the map file than its size");
                set_passable(n);
                [[fallthrough]];
            default:
                n++;
                break;
        }
    }
    n += width + 1; // Should be +2 but already counted a +1 from the previous \n
    ASSERT(n == size(), "Unexpected number of cells");

    // Close file.
    map_file.close();
}

void Map::resize(const Position width, const Position height)
{
    DEBUG_ASSERT(empty());
    passable_.resize(width * height, false);
    width_ = width;
    height_ = height;
}

void Map::set_passable(const Node n)
{
    DEBUG_ASSERT(n < size());
    passable_[n] = true;
}

void Map::set_obstacle(const Node n)
{
    DEBUG_ASSERT(n < size());
    passable_[n] = false;
}

void Map::print() const
{
    for (Node n = 0; n < size(); ++n)
    {
        if (n % width() == 0)
        {
            fmt::print("\n");
        }
        fmt::print("{}", (*this)[n] ? '.' : '@');
    }
    fmt::print("\n");
    fflush(stdout);
}
