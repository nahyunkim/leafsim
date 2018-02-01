#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, float thickness) {
  this->width = width;
  this->height = height;
  this->thickness = thickness;
  this->pixel_indices = std::map<int,int>();

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  Matrix4x4 transform = Matrix4x4::identity();
  for (vector<double> matrix_data : transform_data) {
    transform = transform * Matrix4x4(&matrix_data[0]);
    cout << transform << endl;
  }

  int count = 0;
  for (int j = 0; j < num_height_points; j++) {
    for (int i = 0; i < num_width_points; i++) {
      if (leaf_grid[j][i] > 0) {
        Vector3D pos;
        if (orientation == HORIZONTAL) {
          pos = Vector3D(i / (float(num_width_points) * width), 1, j / float(num_height_points) * height);
        } else {
          float offset = (rand() % 2000 - 1000) / float(1000000);
          pos = Vector3D(i / (float(num_width_points) * width), j / float(num_height_points) * height, offset);
        }
        bool pin_this = false;
        for (vector<int> pin : pinned) {
          if (pin[0] == i && pin[1] == j) {
            pin_this = true;
          }

        }
        this->pixel_indices[j * num_width_points + i] = count;
        Vector4D transpos = transform * pos;
        point_masses.push_back(PointMass(Vector3D(transpos[0], transpos[1], transpos[2]), pin_this));
        count++;
      }
    }
  }

  //basic springs 
  for (int j = 0; j < num_height_points; j++) {
    for (int i = 0; i < num_width_points; i++) {
        int curr = j * num_width_points + i;
        int up = curr - num_width_points;
        int left = curr - 1;
        int upper_left = curr - num_width_points - 1;
        int upper_right = curr - num_width_points + 1;
        int two_right = curr + 2;
        int two_above = curr - (2 * num_width_points);
        if (leaf_grid[j][i] > 0) {
          if (i > 0 && leaf_grid[j][i-1] > 0) {
            springs.push_back(Spring(&point_masses[pixel_indices[left]], &point_masses[pixel_indices[curr]], STRUCTURAL));
          }
          if (j > 0 && leaf_grid[j-1][i] > 0) {
            springs.push_back(Spring(&point_masses[pixel_indices[up]], &point_masses[pixel_indices[curr]],STRUCTURAL));
          }
          if (i > 0 && j > 0 && leaf_grid[j-1][i-1] > 0) {
            springs.push_back(Spring(&point_masses[pixel_indices[upper_left]], &point_masses[pixel_indices[curr]], SHEARING));
          }
          if (i < num_width_points - 1 && j > 0 && leaf_grid[j-1][i+1] > 0) {
            springs.push_back(Spring(&point_masses[pixel_indices[upper_right]], &point_masses[pixel_indices[curr]],SHEARING));
          }
          if (i < num_width_points - 2 && leaf_grid[j][i+2] > 0) {
            springs.push_back(Spring(&point_masses[pixel_indices[two_right]], &point_masses[pixel_indices[curr]],BENDING));
          }
          if (j > 1 && leaf_grid[j-2][i] > 0) {
            springs.push_back(Spring(&point_masses[pixel_indices[two_above]], &point_masses[pixel_indices[curr]],BENDING));
          }
        }
      }
    }


    for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
        int curr = j * num_width_points + i;
        int neighbor;
        if (leaf_grid[j][i] == 2) {
          neighbor = findNearest(2, i, j, 2);
          if (neighbor != -1) {
            springs.push_back(Spring(&point_masses[pixel_indices[curr]], &point_masses[pixel_indices[neighbor]], VEIN));
          }
        }

        if (leaf_grid[j][i] == 3) {
          neighbor = findNearest(3, i, j, 3);
          if (neighbor != -1) {
            springs.push_back(Spring(&point_masses[pixel_indices[curr]], &point_masses[pixel_indices[neighbor]], SIDEVEIN));
          }

          neighbor = findNearest(2, i, j, 2);
          if (neighbor != -1) {
            springs.push_back(Spring(&point_masses[pixel_indices[curr]], &point_masses[pixel_indices[neighbor]], SIDEVEIN));
          }

          neighbor = findNearest(2, i, j, 4);
          if (neighbor != -1) {
            springs.push_back(Spring(&point_masses[pixel_indices[curr]], &point_masses[pixel_indices[neighbor]], SIDEVEIN));
          }
        }

        if (leaf_grid[j][i] == 4) {
          // neighbor = findNearest(4, i, j, 4);
          neighbor = findFarthest(i, j, 4);
          if (neighbor != -1) {
            springs.push_back(Spring(&point_masses[pixel_indices[curr]], &point_masses[pixel_indices[neighbor]], MARGIN));
          }
        }
      }
    }

}
//finds the index of the nearest connectable thing in range
//
int Cloth::findNearest(int range, int pm_i, int pm_j, int target) {
  int max_j = min(num_height_points, pm_j + range);
  int min_j = max(0, pm_j - range);
  int max_i = min(num_width_points, pm_i + range);
  int min_i = max(0, pm_i - range);

    for (int j = min_j; j < max_j; j++) {
      for (int i = min_i; i < max_i; i++) {
        if (leaf_grid[j][i] == target && (j != pm_j && i != pm_i)) { //prevent self-springs
          return j * num_width_points + i;
        }
      }
    }
    return -1;
}

//finds the index of the farthest target point available
//
int Cloth::findFarthest(int pm_i, int pm_j, int target) {
    double max_distance = 0.0;
    int max_index = -1;
    for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
        if (leaf_grid[j][i] == target && (j != pm_j && i != pm_i)) { //prevent self-springs
          if (max_distance < sqrt(pow(pm_i - i, 2) + pow(pm_j - j, 2))) {
            max_distance = sqrt(pow(pm_i - i, 2) + pow(pm_j - j, 2));
            max_index = j * num_width_points + i;
          }
        }
      }
    }
    return max_index;
}


void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  
  //external accel
  Vector3D total_accels = Vector3D();
  for (Vector3D acc : external_accelerations) {
    total_accels += acc;
  }
  for (PointMass &pm : point_masses) {
    pm.forces = mass * total_accels;
  }

  //spring force
  for (Spring &s : springs) {
    if ((s.spring_type == STRUCTURAL && cp->enable_structural_constraints ) || 
      (s.spring_type == SHEARING && cp->enable_shearing_constraints ) || 
      (s.spring_type == BENDING && cp->enable_bending_constraints ) ) {
          double spring_force = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
        Vector3D direction = (s.pm_a->position - s.pm_b->position).unit();
      s.pm_a->forces += spring_force * -direction;
      s.pm_b->forces += spring_force * direction;
    }
    else if (s.spring_type == VEIN) {
      double spring_force = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length) * 10;
        Vector3D direction = (s.pm_a->position - s.pm_b->position).unit();
      s.pm_a->forces += spring_force * -direction;
      s.pm_b->forces += spring_force * direction;     
    }
    else if (s.spring_type == SIDEVEIN) {
      double spring_force = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length)* 10;
        Vector3D direction = (s.pm_a->position - s.pm_b->position).unit();
      s.pm_a->forces += spring_force * -direction;
      s.pm_b->forces += spring_force * direction;     
    }
    else if (s.spring_type == MARGIN) {
      double spring_force = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length)* 10;
        Vector3D direction = (s.pm_a->position - s.pm_b->position).unit();
      s.pm_a->forces += spring_force * -direction;
      s.pm_b->forces += spring_force * direction;     
    }
  }

    //apply force
    for (PointMass &pm : point_masses) {
        if (!pm.pinned) {
    Vector3D new_pos = pm.position + (1 - (cp->damping / 100))
    * (pm.position - pm.last_position) + pm.forces/ mass * pow(delta_t, 2);
    pm.last_position = pm.position;
    pm.position = new_pos;
    }
    pm.forces = Vector3D();
  }

  // TODO (Part 4): Handle self-collisions.
  // This won't do anything until you complete Part 4.
  build_spatial_map();
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  // This won't do anything until you complete Part 3.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects) {
      co->collide(pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring &s : springs) {
    Vector3D direction = (s.pm_a->position - s.pm_b->position);
    double curr_length = direction.norm();
    if (curr_length > s.rest_length * 1.10) {
      if (s.pm_a->pinned) {
        s.pm_b->position = s.pm_a->position - (direction.unit() * s.rest_length * 1.10);
      } else if (s.pm_b->pinned) {
        s.pm_a->position = s.pm_b->position + (direction.unit() * s.rest_length * 1.10);
      } else {
        Vector3D midpoint = s.pm_a->position - (direction / 2);
        Vector3D posb = midpoint - direction.unit() * (s.rest_length * 1.10) /  2;
        Vector3D posa = midpoint + direction.unit() * (s.rest_length * 1.10) /  2;
        s.pm_b->position = posb;
        s.pm_a->position = posa;
      }
    }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();
  // TODO (Part 4): Build a spatial map out of all of the point masses.
  // for (PointMass &pm : point_masses) {
  //   float key = hash_position(pm.position);
  //   if (map[key] == NULL)
  //     map[key] = new vector<PointMass *>();
  //   map[key]->push_back(&pm);
  // }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  // Vector3D correction;
  // vector<PointMass *>* vec = map[hash_position(pm.position)];
  // int count_corrections = vec->size() - 1;
  // int count = 0;
  // for (PointMass *&otherpm : *vec) {
  //   Vector3D distance = (pm.position - otherpm->position);
  //     if (otherpm != &pm && distance.norm() < 2 * thickness) {
  //       correction += distance.unit() * ( 2 * thickness - distance.norm());
  //       count++;
  //     }
  // }

  //   if (count > 0) {
  //   pm.position = pm.position + (correction/ count / simulation_steps);
  // }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents
  // membership in some uniquely identified 3D box volume.
  // stringstream potatos;
  // potatos << floor(pos.x / (3 * width / num_width_points)) << 'x' << floor(pos.y / (3 * height / num_height_points)) << 'y' << floor(pos.z / max(3 * width / num_width_points, 3 * height / num_height_points)) << 'z';
  // return hash<string>()(potatos.str());
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  std::map<int,int> top_left_triangles;
  std::map<int,int> bottom_right_triangles;

  std::map<int,int> bottom_left_triangles;
  std::map<int,int> top_right_triangles;

  int count = 0;
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      if (leaf_grid[y+1][x] && leaf_grid[y][x+1]) {
        PointMass *pm_down = &point_masses[this->pixel_indices[(y + 1) * num_width_points + x]];
        PointMass *pm_right = &point_masses[this->pixel_indices[y * num_width_points + x + 1]];

        if (leaf_grid[y][x]) { // top left
          PointMass *pm = &point_masses[this->pixel_indices[y * num_width_points + x]];
          triangles.push_back(new Triangle(pm, pm_down, pm_right));
          top_left_triangles[y * num_width_points + x] = count;
          count++;
        }
        if (leaf_grid[y][x+1] && leaf_grid[y+1][x] && leaf_grid[y+1][x+1]) { // bottom right
          PointMass *pm_down = &point_masses[this->pixel_indices[(y + 1) * num_width_points + x]];
          PointMass *pm_right = &point_masses[this->pixel_indices[y * num_width_points + x + 1]];
          //bottom right to top right
          PointMass *pm_down_right = &point_masses[this->pixel_indices[(y + 1) * num_width_points + x + 1]];
          triangles.push_back(new Triangle(pm_right, pm_down, pm_down_right));
          bottom_right_triangles[y * num_width_points + x] = count;
          count++;
        }
      }

      else if (leaf_grid[y][x] && leaf_grid[y+1][x] && leaf_grid[y+1][x+1]) { // bottom left
        PointMass *pm = &point_masses[this->pixel_indices[y * num_width_points + x]];
        PointMass *pm_down = &point_masses[this->pixel_indices[(y + 1) * num_width_points + x]];
        PointMass *pm_down_right = &point_masses[this->pixel_indices[(y + 1) * num_width_points + x + 1]];
        triangles.push_back(new Triangle(pm, pm_down, pm_down_right));
        bottom_left_triangles[y * num_width_points + x] = count;
        count++;
      }

      else if (leaf_grid[y][x] && leaf_grid[y][x+1] && leaf_grid[y+1][x+1]) { // top right
        PointMass *pm = &point_masses[this->pixel_indices[y * num_width_points + x]];
        PointMass *pm_right = &point_masses[this->pixel_indices[y * num_width_points + x + 1]];
        PointMass *pm_down_right = &point_masses[this->pixel_indices[(y + 1) * num_width_points + x + 1]];
        triangles.push_back(new Triangle(pm, pm_down_right, pm_right));
        top_right_triangles[y * num_width_points + x] = count;
        count++;
      }
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      Triangle *top_left_tri = nullptr;
      int pm_coor = y * num_width_points + x;
      if (top_left_triangles.find(pm_coor) != top_left_triangles.end()) {
        top_left_tri = triangles[top_left_triangles[pm_coor]];
        int left = pm_coor-1;
        if (x > 0 && leaf_grid[y][x-1] > 0 && 
            bottom_right_triangles.find(left) != bottom_right_triangles.end()) {
          top_left_tri->pm1->halfedge->twin = triangles[bottom_right_triangles[left]]->pm3->halfedge;
        }
        else {
          top_left_tri->pm1->halfedge->twin = nullptr;
        }
        int top = pm_coor-num_width_points;
        if (y > 0 && leaf_grid[y-1][x] > 0 && 
            bottom_right_triangles.find(top) != bottom_right_triangles.end()) {
          top_left_tri->pm3->halfedge->twin = triangles[bottom_right_triangles[top]]->pm2->halfedge;
        }
        else {
          top_left_tri->pm3->halfedge->twin = nullptr;
        }
        top_left_tri->pm2->halfedge->twin = nullptr;
      }
      if (bottom_right_triangles.find(pm_coor) != bottom_right_triangles.end()) {
        Triangle *bottom_right_tri = triangles[bottom_right_triangles[pm_coor]];
        if (top_left_tri != nullptr) {
          bottom_right_tri->pm1->halfedge->twin = top_left_tri->pm2->halfedge;
          top_left_tri->pm2->halfedge->twin = bottom_right_tri->pm1->halfedge;
        }
        int right = pm_coor + 1;
        if (x < num_width_points - 1 && leaf_grid[y][x+1] && 
            top_left_triangles.find(right) != top_left_triangles.end()) {
          bottom_right_tri->pm3->halfedge->twin = triangles[top_left_triangles[right]]->pm1->halfedge;
        }
        else {
          bottom_right_tri->pm3->halfedge->twin = nullptr;
        }
        int bottom = pm_coor + num_width_points;
        if (y < num_height_points - 1 && leaf_grid[y+1][x] &&
            top_left_triangles.find(bottom) != top_left_triangles.end()) {
          bottom_right_tri->pm2->halfedge->twin = triangles[top_left_triangles[bottom]]->pm3->halfedge;
        }
        else {
          bottom_right_tri->pm2->halfedge->twin = nullptr;
        }
      }
      if (bottom_left_triangles.find(pm_coor) != bottom_left_triangles.end()) {
        Triangle *bottom_left_tri = triangles[bottom_left_triangles[pm_coor]];
        bottom_left_tri->pm1->halfedge->twin = triangles[bottom_right_triangles[pm_coor-1]]->pm3->halfedge;
        bottom_left_tri->pm2->halfedge->twin = triangles[top_left_triangles[pm_coor+num_width_points]]->pm3->halfedge;
        bottom_left_tri->pm3->halfedge->twin = nullptr;
      }
      if (top_right_triangles.find(pm_coor) != top_right_triangles.end()) {
        Triangle * top_right_tri = triangles[top_right_triangles[pm_coor]];
        top_right_tri->pm2->halfedge->twin = triangles[top_left_triangles[pm_coor+1]]->pm1->halfedge;
        top_right_tri->pm3->halfedge->twin = triangles[bottom_right_triangles[pm_coor-num_width_points]]->pm2->halfedge;
        top_right_tri->pm1->halfedge->twin = nullptr;
      }
    }
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
