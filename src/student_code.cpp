#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
    /**
     * Computes the linear interpolation of two vectors and a scalar.
     * @param p0
     * @param p1
     * @param t
     * @return new weighted vector
     */
    Vector2D lerp2D(Vector2D p0, Vector2D p1, float t) {
        return (1 - t) * p0 + t * p1;
    }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    int num_points = points.size();
    std::vector<Vector2D> new_points = std::vector<Vector2D>();
    for (int i = 0; i < num_points - 1; i++) {
        new_points.push_back(lerp2D(points[i], points[i + 1], this->t));
    }
    return new_points;
  }

    /**
       * Computes the linear interpolation of two vectors and a scalar.
       * @param p0
       * @param p1
       * @param t
       * @return new weighted vector
       */
    Vector3D lerp3D(Vector3D p0, Vector3D p1, double t) {
        return (1 - t) * p0 + t * p1;
    }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    int num_points = points.size();
    std::vector<Vector3D> final_points = std::vector<Vector3D>();
    for (int i = 0; i < num_points; i++) {
        final_points.push_back(lerp3D(points[i], points[i+1], t));
    }
    return final_points;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    int num_points = points.size();
    std::vector<Vector3D> new_points = BezierPatch::evaluateStep(points, t);
    for (int i = 0; i < num_points - 2; i++) {
        new_points = BezierPatch::evaluateStep(new_points, t);
    }
    return new_points[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    int num_points = controlPoints.size();
    std::vector<Vector3D> final_points = std::vector<Vector3D>();
    for (int i = 0; i < num_points; i++) {
        final_points.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(final_points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      Vector3D n = Vector3D(0, 0, 0);
      HalfedgeCIter  h = this->halfedge();
      do
         {
            Vector3D a = this->position;
            Vector3D b = h->twin()->vertex()->position;
            Vector3D c = h->next()->next()->vertex()->position;
            n += (cross(b-a,c-a)*cross(b-a,c-a).norm());
            h = h->next()->next();
            h = h->twin();
            }
          while( h != this->halfedge() );
    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->halfedge()->isBoundary() || e0->halfedge()->twin()->isBoundary())
          return e0;

      //Initialize Half-edges
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      //Initialize vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      //Initialize faces
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      //Initialize edges
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      //Set Neighbors
      h0->setNeighbors( h5 , h3, v2, e0, f0);
      h3->setNeighbors( h2 , h0, v3, e0, f1);
      h5->setNeighbors( h1 , h9, v3, e4, f0);
      h1->setNeighbors( h0 , h6, v1, e1, f0);
      h2->setNeighbors( h4 , h7, v2, e2, f1);
      h4->setNeighbors( h3 , h8, v0, e3, f1);

      //Set vertices' half-edges
      v0->halfedge() = h4;
	  v1->halfedge() = h1;
      v2->halfedge() = h0;
	  v3->halfedge() = h3;

	  //Set faces' half-edges
      f0->halfedge() = h0;
      f1->halfedge() = h3;

      //Set edges' half-edges
      e1->halfedge() = h1;
      e0->halfedge() = h0;
      e4->halfedge() = h5;
      e3->halfedge() = h4;
      e2->halfedge() = h2;

      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->halfedge()->isBoundary() || e0->halfedge()->twin()->isBoundary())
          return e0->halfedge()->vertex();

      //Initialize hals-edges
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      //Initialize vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      //Initialize face
      FaceIter f0 = h0->face();

      HalfedgeIter hn0 = newHalfedge();
      HalfedgeIter hn1 = newHalfedge();
      HalfedgeIter hn2 = newHalfedge();
      HalfedgeIter hn3 = newHalfedge();
      HalfedgeIter hn4 = newHalfedge();
      HalfedgeIter hn5 = newHalfedge();

      EdgeIter e1 = newEdge();
      EdgeIter e2 = newEdge();
      EdgeIter e3 = newEdge();

      FaceIter f1 = newFace();
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();

      VertexIter v4 = newVertex();

      v4->position = (v0->position + v1->position) * 0.50;
      v4->halfedge() = hn0;

      h0->setNeighbors( hn2 , hn3, v0, e0, f0);
      h3->setNeighbors( hn4 , hn0, v1, e1, f3);
      h5->setNeighbors( h3 , h5->twin(), v3, h5->edge(), f3);
      h1->setNeighbors( hn1 , h1->twin(), v1, h1->edge(),f1);
      h2->setNeighbors( h0 , h2->twin(), v2, h2->edge(),f0);
      h4->setNeighbors( hn5 , h4->twin(), v0, h4->edge(),f2);
      hn0->setNeighbors( h1 , h3, v4, e1, f1);
      hn1->setNeighbors( hn0 , hn2, v2, e2, f1);
      hn2->setNeighbors( h2 , hn1, v4, e2, f0);
      hn3->setNeighbors( h4 , h0, v4, e0, f2);
      hn4->setNeighbors( h5 , hn5, v4, e3, f3);
      hn5->setNeighbors( hn3 , hn4, v3, e3, f2);

      f0->halfedge() = h0;
      f1->halfedge() = hn0;
      f2->halfedge() = hn3;
      f3->halfedge() = h3;

      e1->halfedge() = hn0;
      e2->halfedge() = hn1;
      e3->halfedge() = hn4;
      e0->halfedge() = h0;

      v0->halfedge() = h0;
      v1->halfedge() = h3;
      v2->halfedge() = h2;
      v3->halfedge() = h5;

      return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.

    //Update position of old vertices
      for ( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
          v->isNew = false;
          //Create weights
          double n = v->degree();
          double u = 3.0 / (8.0 * n);
          if (n == 3) {
              u = 3.0 / 16.0;
          }

          Vector3D neighbor_pos_sum(0.0,0.0,0.0);
          HalfedgeCIter h = v->halfedge();

          //Sum the position of each neighboring vertex
          do {
              h = h->twin();
              neighbor_pos_sum += h->vertex()->position;
              h = h->next();
          } while (h!= v->halfedge());

          v->newPosition = (1 - n * u) * v->position + u * neighbor_pos_sum;
      }

      //Update position of new vertices
      for ( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ ) {

          Vector3D v0 = e->halfedge()->vertex()->position;
          Vector3D v1 = e->halfedge()->twin()->vertex()->position;
          Vector3D v2 = e->halfedge()->next()->next()->vertex()->position;
          Vector3D v3 = e->halfedge()->twin()->next()->next()->vertex()->position;

          e->newPosition = 3.0 / 8.0 * (v0 + v1) + 1.0 / 8.0 * (v2 + v3);
          e->isNew = false;
      }

      //Split each edge in mesh
      int size = mesh.nEdges();
      int i = 0;
      for( EdgeIter e = mesh.edgesBegin(); i < size;  i++ ) {
          if(!e->isNew) {
              VertexIter v = mesh.splitEdge(e);
              v->isNew = true;
              v->newPosition = e->newPosition;
              HalfedgeIter h = v->halfedge();
              h->edge()->isNew = false;
              h->twin()->next()->twin()->next()->edge()->isNew = false;
              h->twin()->next()->edge()->isNew = true;
              h->next()->next()->edge()->isNew = true;
          }
          e++;
      }

      //Flip new edges that connect an old and new vertices
      for ( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ ) {
          if (e->isNew) {
              HalfedgeIter h = e->halfedge();
              HalfedgeIter t = h->twin();

              VertexIter v0 = h->vertex();
              VertexIter v1 = t->vertex();
              if((!v0->isNew && v1->isNew) || (v0->isNew && !v1->isNew)) {
                  mesh.flipEdge(e);
              }
          }
      }

      //Update vertices to their new positions
      for ( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->position = v->newPosition;
      }
  }
}