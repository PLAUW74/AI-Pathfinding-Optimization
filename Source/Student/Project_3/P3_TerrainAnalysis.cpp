#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float distance_to_closest_wall(int row, int col)
{
    /*
        Check the euclidean distance from the given cell to every other wall cell,

        with cells outside the map bounds treated as walls, (???)
        
        and return the smallest distance.  

        Make use of the is_valid_grid_position and is_wall member
        functions in the global terrain to determine if a cell is within map bounds
        and a wall, respectively.
    */

    // WRITE YOUR CODE HERE
    float min_distance = std::numeric_limits<float>::max(); //big number

    //// Get the terrain dimensions 
    //int max_rows = 40; 
    //int max_cols = 40;
    //// Check all possible positions in an expanded grid that includes out-of-bounds
    //// We need to check beyond the map boundaries since out-of-bounds are walls
    //int search_radius = std::max(max_rows, max_cols);

    // Check cells
    for (int r = -1; r <= terrain.get()->get_map_height(); r++) {
        for (int c = -1; c <= terrain.get()->get_map_height(); c++) {
            // Skip the current position
            if (r == row && c == col) {
                continue;
            }

            bool is_wall_cell = false;

            // Check if position is out of bounds (treated as wall)
            if (!terrain->is_valid_grid_position(r, c)) {
                is_wall_cell = true;
            }
            // Check if position is within bounds but is a wall
            else if (terrain->is_wall(r, c)) {
                is_wall_cell = true;
            }

            if (is_wall_cell) {
                // Calculate euclidean distance
                float dx = static_cast<float>(c - col);
                float dy = static_cast<float>(r - row);
                float distance = std::sqrt(dx * dx + dy * dy);

                // Update minimum distance
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }
    }

    return min_distance;
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  
        
        You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  
        
        Make use of the line_intersect helper function for the intersection test and the is_wall member
        function in the global terrain to determine if a cell is a wall or not.
    */

    // WRITE YOUR CODE HERE

    // Calculate centerpoints of the two cells
    float x0 = col0 + 0.5f;
    float y0 = row0 + 0.5f;
    float x1 = col1 + 0.5f;
    float y1 = row1 + 0.5f;

    // Small epsilon value to "puff out" wall boundaries
    const float epsilon = 1e-3f;

    // Get terrain dimensions to determine search area
    int max_rows = terrain.get()->get_map_height(); // Adjust method name as needed
    int max_cols = terrain.get()->get_map_width();   // Adjust method name as needed

    // Determine bounding box for the line segment to optimize search
    int minRow = std::min(row0, row1);
    int maxRow = std::max(row0, row1);
    int minCol = std::min(col0, col1);
    int maxCol = std::max(col0, col1);

    // Check every wall cell in the bounding rectangle
    for (int r = minRow; r <= maxRow; r++) {
        for (int c = minCol; c <= maxCol; c++) {
            // Skip if this cell is not a wall
            if (!terrain->is_wall(r, c)) {
                continue;
            }

            // Define the four boundary lines of the wall cell (puffed out by epsilon)
            float left = c - epsilon;
            float right = c + 1.0f + epsilon;
            float bottom = r - epsilon;
            float top = r + 1.0f + epsilon;

            // Check intersection with each of the four wall boundaries
            Vec2 lineStart(x0, y0);
            Vec2 lineEnd(x1, y1);

            // Top boundary (horizontal line from (left, top) to (right, top))
            if (line_intersect(lineStart, lineEnd, Vec2(left, top), Vec2(right, top))) {
                return false;
            }
            // Bottom boundary (horizontal line from (left, bottom) to (right, bottom))
            if (line_intersect(lineStart, lineEnd, Vec2(left, bottom), Vec2(right, bottom))) {
                return false;
            }
            // Left boundary (vertical line from (left, top) to (left, bottom))
            if (line_intersect(lineStart, lineEnd, Vec2(left, top), Vec2(left, bottom))) {
                return false;
            }
            // Right boundary (vertical line from (right, top) to (right, bottom))
            if (line_intersect(lineStart, lineEnd, Vec2(right, top), Vec2(right, bottom))) {
                return false;
            }
        }
    }

    // If no intersections found, path is clear
    return true;
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        - Mark every cell in the given layer with the value 1 / (d * d), where d is the distance to the closest wall or edge.  
        - Make use of the distance_to_closest_wall helper function.  
        - Walls should not be marked.
    */

    // WRITE YOUR CODE HERE

    // Iterate through every cell in the layer
    for (int row = 0; row < terrain.get()->get_map_height(); row++) {
        for (int col = 0; col < terrain.get()->get_map_width(); col++) {

            // Skip wall cells
            if (terrain->is_wall(row, col)) {
                continue;
            }

            // Calculate distance to closest wall 
            float distance = distance_to_closest_wall(row, col);

            // Calculate openness value: 1 / (d * d)
            float openness = 1.0f / (distance * distance);

            // Set the value in the layer
            layer.set_value(row, col, openness);
        }
    }
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
    int height = terrain.get()->get_map_height();
    int width = terrain.get()->get_map_width();

    // Iterate through every cell in the layer
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            // Skip wall cells - they should not be marked
            if (terrain->is_wall(row, col)) {
                continue;
            }

            int visible_count = 0;

            // Check visibility to every other cell
            for (int target_row = 0; target_row < height; target_row++) {
                for (int target_col = 0; target_col < width; target_col++) {
                    // Skip if checking against self
                    if (target_row == row && target_col == col) {
                        continue;
                    }

                    // Skip if target is a wall (can't see walls)
                    if (terrain->is_wall(target_row, target_col)) {
                        continue;
                    }

                    // Check if there's a clear path between the two cells
                    if (is_clear_path(row, col, target_row, target_col)) {
                        visible_count++;
                    }
                }
            }

            // Calculate visibility value: count / 160
            float visibility = static_cast<float>(visible_count) / 160.0f;

            // Cap the value at 1.0
            if (visibility > 1.0f) {
                visibility = 1.0f;
            }

            // Set the value in the layer
            layer.set_value(row, col, visibility);
        }
    }

}

void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 
        - 1.0 if it is visible to the given cell, 
        - 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell. Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE

    int height = terrain.get()->get_map_height();
    int width = terrain.get()->get_map_width();

    // First pass: Mark all cells as visible (1.0) or not visible (0.0)
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            // Skip wall cells - they should remain 0.0
            if (terrain->is_wall(r, c)) {
                layer.set_value(r, c, 0.0f);
                continue;
            }

            // Check if this cell is visible to the target cell
            if (is_clear_path(row, col, r, c)) {
                layer.set_value(r, c, 1.0f);  // Visible
            }
            else {
                layer.set_value(r, c, 0.0f);  // Not visible (will be updated to 0.5 if adjacent)
            }
        }
    }

    // Second pass: Mark cells adjacent to visible cells as 0.5
    // Store which cells should be 0.5 to avoid modifying during iteration
    std::vector<std::pair<int, int>> adjacent_cells;

    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            // Only consider cells that are currently 0.0 (not visible and not walls)
            if (layer.get_value(r, c) == 0.0f && !terrain->is_wall(r, c)) {
                
                // Check all 8 adjacent cells (including diagonals)
                bool adjacent_to_visible = false;
                //Check offset
                for (int dr = -1; dr <= 1; dr++) {
                    for (int dc = -1; dc <= 1; dc++) {
                        // Skip the center cell
                        if (dr == 0 && dc == 0) continue;

                        //Neighbor
                        int adj_row = r + dr;
                        int adj_col = c + dc;

                        // Check bounds
                        if (adj_row >= 0 && adj_row < height && adj_col >= 0 && adj_col < width) {
                            // If adjacent cell is visible (1.0) AND 
                            // not a wall
                            // AND there's a clear path between the current cell and the adjacent visible cell
                            if (layer.get_value(adj_row, adj_col) == 1.0f &&
                                !terrain->is_wall(adj_row, adj_col) &&
                                is_clear_path(r, c, adj_row, adj_col)) {
                                adjacent_to_visible = true;
                                break;
                            }
                        }
                    }
                    if (adjacent_to_visible) break;
                }

                if (adjacent_to_visible) {
                    adjacent_cells.push_back({ r, c });
                }
            }
        }
    }

    // Apply the 0.5 values to adjacent cells
    for (const auto& cell : adjacent_cells) {
        layer.set_value(cell.first, cell.second, 0.5f);
    }
}

void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
    /*
        For every cell in the given layer that is visible to the given agent,
        mark it as 1.0, otherwise don't change the cell's current value.

        You must consider the direction the agent is facing.  All of the agent data is
        in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

        Take the dot product between the view vector and the vector from the agent to the cell,
        both normalized, 
        and compare the cosines directly instead of taking the arccosine to
        avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

        Give the agent a field of view slighter larger than 180 degrees.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE

    // Get layer dimensions
    int height = terrain.get()->get_map_height();
    int width = terrain.get()->get_map_width();

    // Get agent position in grid coordinates
    GridPos agent_pos = terrain.get()->get_grid_position(agent->get_position());
    int agent_row = (int)agent_pos.row;    
    int agent_col = (int)agent_pos.col;    

    // Get agent's facing direction (normalized view vector in XZ plane)
    Vec3 agent_forward = agent->get_forward_vector();
    Vec2 view_vector(agent_forward.z, agent_forward.x);  // Project to XZ plane
    view_vector.Normalize();

    // Field of view slightly larger than 180 degrees
    float fov_cosine_threshold = -0.02f;

    // Iterate through every cell in the layer
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {

            // Skip wall cells - they should not be marked as visible
            if (terrain->is_wall(r, c)) {
                continue;  // Don't change wall cell values
            }

            // Calculate vector from agent to current cell center
            Vec2 cell_center(c + 0.5f, r + 0.5f);
            Vec2 agent_grid(agent_col + 0.5f, agent_row + 0.5f);
            Vec2 agent_to_cell = cell_center - agent_grid;
            agent_to_cell.Normalize(); // Normalize the vector from agent to cell

            // Calculate dot product (cosine of angle)
            float cosine = agent_to_cell.Dot(view_vector);

            // Check if cell is within field of view
            if (cosine > fov_cosine_threshold) {
                // Check if there's a clear line of sight using the helper function
                // Use world coordinates for both agent and cell center

                if (is_clear_path(agent_row, agent_col, r, c)) {
                    layer.set_value(r, c, 1.0f);
                }

            }

        }
    }
}


void propagate_solo_occupancy(MapLayer<float>& layer, float decay, float growth)
{
    /*
        For every cell in the given layer:
            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.
        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    // Get layer dimensions
    int height = terrain.get()->get_map_height();
    int width = terrain.get()->get_map_width();
    // Create temporary layer
    float temp_layer[40][40];
    // Process every cell
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            // Skip processing if this cell is a wall
            if (terrain.get()->is_wall(r, c)) {
                temp_layer[r][c] = 0.0f;
                continue;
            }
            float max_neighbor_value = 0.0f;
            // Check all 8 neighbors (and handle boundary conditions)
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    // Skip the center cell (not a neighbor)
                    if (dr == 0 && dc == 0) continue;
                    int neighbor_r = r + dr;
                    int neighbor_c = c + dc;
                    // Check if neighbor is within bounds
                    if (neighbor_r >= 0 && neighbor_r < height &&
                        neighbor_c >= 0 && neighbor_c < width) {
                        // Check if the neighboring cell is a wall - if so, don't spread to it
                        if (!terrain.get()->is_wall(neighbor_r, neighbor_c)) {
                            // For diagonal neighbors, check if we can actually reach them
                            // (i.e., at least one of the two adjacent cells is not a wall)
                            bool can_reach = true;
                            if (dr != 0 && dc != 0) { // This is a diagonal neighbor
                                // Check the two adjacent cells that form the path to this diagonal
                                bool horizontal_blocked = terrain.get()->is_wall(r, neighbor_c);
                                bool vertical_blocked = terrain.get()->is_wall(neighbor_r, c);
                                // If both paths are blocked, we can't reach this diagonal neighbor
                                if (horizontal_blocked && vertical_blocked) {
                                    can_reach = false;
                                }
                            }

                            if (can_reach) {
                                // Calculate distance to neighbor
                                float distance = sqrt(dr * dr + dc * dc);
                                // Get neighbor value and apply exponential decay
                                float neighbor_value = layer.get_value(neighbor_r, neighbor_c) * exp(-1.0f * distance * decay);
                                // Keep the highest decayed neighbor value
                                if (neighbor_value > max_neighbor_value) {
                                    max_neighbor_value = neighbor_value;
                                }
                            }
                        }
                    }
                }
            }
            // Get current cell value
            float current_value = layer.get_value(r, c);
            // Linearly interpolate between current value and max neighbor value
            float new_value = lerp(current_value, max_neighbor_value, growth);
            // Store in temporary layer
            temp_layer[r][c] = new_value;
        }
    }
    // Copy temporary layer back to the original layer
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            layer.set_value(r, c, temp_layer[r][c]);
        }
    }
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].  Negative values should be left unmodified.
    */

    // WRITE YOUR CODE HERE

     // Get layer dimensions
    int height = terrain.get()->get_map_height();
    int width = terrain.get()->get_map_width();

    // Find the maximum value in the layer
    float max_value = 0.0f;
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            float current_value = layer.get_value(r, c);
            if (current_value > max_value) {
                max_value = current_value;
            }
        }
    }

    // If max_value is 0 or very close to 0, avoid division by zero
    if (max_value <= 0.0f) {
        return; // Nothing to normalize
    }

    // Normalize all positive values by dividing by max_value
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            float current_value = layer.get_value(r, c);

            // Only normalize positive values, leave negative values unmodified
            if (current_value > 0.0f) {
                float normalized_value = current_value / max_value;
                layer.set_value(r, c, normalized_value);
            }
            // Negative values are left as-is (no modification needed)
        }
    }
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.  
        
        Take the dot product between the view
        vector and the vector from the agent to the cell, both normalized, and compare the
        cosines directly instead of taking the arccosine to avoid introducing floating-point
        inaccuracy (larger cosine means smaller angle).

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */

    // WRITE YOUR CODE HERE

    // Get layer dimensions
    int height = terrain.get()->get_map_height();
    int width = terrain.get()->get_map_width();

    // First, clear out old values by setting any negative value to 0
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            float current_value = layer.get_value(r, c);
            if (current_value < 0.0f) {
                layer.set_value(r, c, 0.0f);
            }
        }
    }

    // Get enemy position and facing direction
    GridPos enemy_pos = terrain.get()->get_grid_position(enemy->get_position());
    Vec3 enemy_facing = enemy->get_forward_vector();
	Vec2 view_vector(enemy_facing.z, enemy_facing.x); 
	view_vector.Normalize();

    // Calculate the cosine of half the FOV angle for comparison
    float half_fov_cosine = cos(fovAngle * 0.5f);

    // Process every cell in the layer
    // Iterate through every cell in the layer
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {

            // Skip wall cells - they should not be marked as visible
            if (terrain->is_wall(r, c)) {
                continue;  // Don't change wall cell values
            }

            // Calculate vector from agent to current cell center
            Vec2 cell_center(c + 0.5f, r + 0.5f);
            Vec2 enemy_grid(enemy_pos.col + 0.5f, enemy_pos.row + 0.5f);
            Vec2 enemy_to_cell = cell_center - enemy_grid;
            enemy_to_cell.Normalize();

            // Calculate dot product (cosine of angle)
            float cosine = enemy_to_cell.Dot(view_vector);

            // Check if cell is within field of view
            if (cosine >= half_fov_cosine) {

                if (is_clear_path(enemy_pos.row, enemy_pos.col, r, c)) {
                    layer.set_value(r, c, occupancyValue);
                }

            }

        }
    }

}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    // WRITE YOUR CODE HERE

    // Get layer dimensions
    int height = terrain.get()->get_map_height();
    int width = terrain.get()->get_map_width();

    // Get enemy position
    GridPos enemy_pos = terrain.get()->get_grid_position(enemy->get_position());

    // Variables to track the best cell
    float highest_value = 0.0f;
    int best_row = -1;
    int best_col = -1;
    float closest_distance = FLT_MAX;
    bool found_target = false;


    // Search through all cells to find the highest nonzero value
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            float current_value = layer.get_value(r, c);

            // Skip zero or negative values
            if (current_value <= 0.0f) {
                continue;
            }

            // Adjust 
            Vec3 cell_pos;
            cell_pos.x = (float)c + 0.5f; // Center of the cell
            cell_pos.z = (float)r + 0.5f; // Center of the cell

            // Calculate distance from enemy to this cell
            float dx = cell_pos.x - (enemy_pos.col + 0.5f);
            float dz = cell_pos.z - (enemy_pos.row + 0.5f);
            float distance = sqrt(dx * dx + dz * dz);

            // Check if this is a better target
            if (current_value > highest_value) {
                // Found a cell with higher value
                highest_value = current_value;
                best_row = r;
                best_col = c;
                closest_distance = distance;
                found_target = true;
            }
            else if (current_value == highest_value && distance < closest_distance) {
                // Found a cell with same highest value but closer distance
                best_row = r;
                best_col = c;
                closest_distance = distance;
            }
        }
    }

    return false; // REPLACE THIS
}
