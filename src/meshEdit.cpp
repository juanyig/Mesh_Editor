#include <float.h>
#include <assert.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"

namespace CMU462 {

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along
  // the edge that was split, rather than the new edges.

  HalfedgeIter h0 = e0->halfedge();
  HalfedgeIter t0 = e0->halfedge()->twin();

  // If the edge is a boundary edge and has a triangle face
  if (e0->isBoundary() && h0->face()->degree() == 3)
  {
    HalfedgeIter h1 = h0->next()->next();
    VertexIter v = addVertexOnEdge(e0);
    addEdge(v->halfedge(), h1);
    return v;
  }

  //if the edge is a boundary edge but does not have a triangle face
  //of if the edge is not a boundary edge and contains a face that is not triangle
  if ((e0->isBoundary() && h0->face()->degree() != 3) ||
      (h0->face()->degree() != 3 || t0->face()->degree() != 3))
  {
    showError("Edge cannot be split. Not triangle mesh");
    return verticesBegin();
  }

  //if the edge is not a boundary edge and contains 2 triangle face on both sides
  HalfedgeIter h1 = h0->next()->next();
  HalfedgeIter h2 = t0->next()->next();
  VertexIter v = addVertexOnEdge(e0); //first add a vertex in the middle and split the edge into 2
  HalfedgeIter h = v->halfedge();
  HalfedgeIter t = h->twin()->next();
  //add a edge between the newly inserted v and 2 vertices of the triangles respectively
  if (h1->face() == h->face())
  {
    addEdge(h1, h);
    addEdge(h2, t);
  }
  else
  {
    addEdge(h1, t);
    addEdge(h2, h);
  }

  return v;
}

VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should collapse the given edge and return an iterator to
  // the new vertex created by the collapse.

  //record e's 2 halfedges, 2 vertices, and 2 faces correspondingly
  HalfedgeIter h = e->halfedge();
  HalfedgeIter t = h->twin();
  HalfedgeIter h_bf, t_bf;
  VertexIter v0 = h->vertex();
  VertexIter v1 = t->vertex();
  FaceIter f0 = h->face();
  FaceIter f1 = t->face();

  // find all adjacent halfedges of v0 and v1
  vector<HalfedgeIter> Adj1, Adj2;
  HalfedgeIter trace1 = h->next();
  HalfedgeIter trace2 = t->next();
  while (trace1 != t)
  {
    Adj1.push_back(trace1);
    if (trace1->twin()->next() == t)
      t_bf = trace1->twin();
    trace1 = trace1->twin()->next();

  }

  while (trace2 != h)
  {
    Adj2.push_back(trace2);
    if (trace2->twin()->next() == h)
      h_bf = trace2->twin();
    trace2 = trace2->twin()->next();
  }

  //assign all adjacent halfedges of v0 to v1
  for (auto adj: Adj2) {
    adj->vertex() = v1;
  }

  //connect the halfedge before h/t to the the one after it
  h_bf->next() = h->next();
  f0->halfedge() = h_bf;
  t_bf->next() = t->next();
  f1->halfedge() = t_bf;
  v1->halfedge() = h->next();

  //delete the edge and its corresponding halfedges and 1 vertex
  deleteEdge(e);
  deleteHalfedge(h);
  deleteHalfedge(t);
  deleteVertex(v0);

  //if the face was originally a triangle, it becomes a single line after collapse
  if (f0->degree() == 2) collapsehelper(h_bf);
  if (f1->degree() == 2) collapsehelper(t_bf);
  return v1;
}

void HalfedgeMesh:: collapsehelper(HalfedgeIter h)
{
  //a helper function for collapseEdge. If a face results in a degree of 2,
  //delete such face and replace it with a single edge
  if (h->face()->degree() != 2)
    return;

  //the face now cotains 2 edge only, keep 1 and delete the other
  //the one that will be deleted/collapsed is denoted by halfedge c
  HalfedgeIter t = h->twin();
  HalfedgeIter c = h->next();
  HalfedgeIter c_t = c->twin();
  VertexIter v1 = t->vertex();
  VertexIter v0 = h->vertex();
  FaceIter f = h->face();

  HalfedgeIter c_t_bf = c_t->next();
  while (c_t_bf->next() != c_t) c_t_bf = c_t_bf->next();
  c_t_bf->next() = h;
  h->next() = c_t->next();
  v1->halfedge() = t;
  v0->halfedge() = h;
  h->face() = c_t_bf->face();
  c_t_bf->face()->halfedge() = h;

  deleteEdge(c->edge());
  deleteHalfedge(c);
  deleteHalfedge(c_t);
  deleteFace(f);
}

VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
  // TODO: (meshEdit)
  // This method should collapse the given face and return an iterator to
  // the new vertex created by the collapse.
  showError("collapseFace() not implemented.");
  return VertexIter();
}

FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
  // TODO: (meshEdit)
  // This method should replace the given vertex and all its neighboring
  // edges and faces with a single face, returning the new face.

  if (v->isBoundary())
  {
      showError("Boundary Vertex cannot be erased");
      return facesBegin();
  }

  //find all adjacent halfedges of vertex v
  vector <HalfedgeIter> adjs;
  HalfedgeIter h = v->halfedge();
  do{
    adjs.push_back(h);
    if (h->twin()->vertex()->degree() == 2){
      showError("Vertex cannot be erased."); // if v is erased, its adjacent vertex will result in a degree of 1
      return facesBegin();
    }
    h = h->twin()->next();
  }while(h != v->halfedge());

  //erase all the adjacent halfedges by calling eraseEdge
  //vertex v will be erased as well inside eraseEdge function
  FaceIter f;
  for (int i = 0; i < adjs.size() - 1; i++)
    f = eraseEdge(adjs[i]->edge());

  return f;
}

EdgeIter HalfedgeMesh::addEdge(HalfedgeIter e0, HalfedgeIter e1) {
// a helper function that adds an edge between two vertices

  if (e0->face() != e1->face())
  {
    showError("not in same face, cannot add edge");
    return EdgeIter();
  }

  //collect all affected halfedges, 2 vertices, and 1 face
  //construct 1 newly inserted edge, 2 halfedges, and 1 face
  HalfedgeIter v0_to_v1 = newHalfedge();
  HalfedgeIter v1_to_v0 = newHalfedge();
  VertexIter v0 = e0->vertex();
  VertexIter v1 = e1->vertex();

  HalfedgeIter e0_bf = e0->next();
  while (e0_bf->next() != e0) e0_bf = e0_bf->next();
  HalfedgeIter e1_bf = e1->next();
  while (e1_bf->next() != e1) e1_bf = e1_bf->next();

  EdgeIter e_add = newEdge();
  e_add->isNew = true;
  FaceIter f0 = e0->face();
  FaceIter f1 = newFace();

  //assign halfedges' vertices, faces, edges, and twins correspondingly
  v0_to_v1->vertex() = v0;
  v1_to_v0->vertex() = v1;
  v0_to_v1->edge() = e_add;
  v1_to_v0->edge() = e_add;
  v0_to_v1->face() = f0;
  v1_to_v0->face() = f1;
  v0_to_v1->twin() = v1_to_v0;
  v1_to_v0->twin() = v0_to_v1;

  e0_bf->next() = v0_to_v1;
  v0_to_v1->next() = e1;
  e1_bf->next() = v1_to_v0;
  v1_to_v0->next() = e0;

  e_add->halfedge() = v0_to_v1;

  //assign face <-> halfedges correctly
  f0->halfedge() = v0_to_v1;
  f1->halfedge() = v1_to_v0;
  HalfedgeIter trace = v1_to_v0;
  do{
    trace->face() = f1;
    trace = trace->next();
  } while (trace != v1_to_v0);

  return e_add;
}

FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should erase the given edge and return an iterator to the
  // merged face.

  //NOTE: this function sometimes crashes, please look at write up for details.

  //if e is a boundary edge, nothing happens
  if (e->isBoundary())
  {
    return e->halfedge()->face();
  }

  HalfedgeIter h = e->halfedge();
  HalfedgeIter t = h->twin();

  vector <EdgeIter> erase;
  HalfedgeIter track1, track2;
  //special case: if there is a hanging edge, delete it
  if (h->face() == t->face())
  {
    HalfedgeIter h0;
    if (h->next() != t) h0 = t;
    else h0 = h;
    HalfedgeIter ht = h0->twin();
    HalfedgeIter hn = ht->next();
    HalfedgeIter hb = h0->next();
    while (hb->next() != h0) hb = hb->next();
    VertexIter v = h0->vertex();
    VertexIter v1 = ht->vertex();
    FaceIter f = h->face();

    hb->next() = hn;
    v->halfedge() = hn;
    f->halfedge() = hn;

    deleteVertex(v1);
    deleteHalfedge(h);
    deleteHalfedge(t);
    deleteEdge(e);
    return f;
  }
  //normal case
  //collect affected components: 2 vertices, 4 halfedges, 2 faces
  HalfedgeIter h_nxt = h->next();
  HalfedgeIter h_bf = h->next();
  while (h_bf->next() != h) h_bf = h_bf->next();

  HalfedgeIter t_nxt = t->next();
  HalfedgeIter t_bf = t_nxt;
  while(t_bf->next() != t) t_bf = t_bf->next();


  VertexIter v0 = h->vertex();
  VertexIter v1 = t->vertex();

  FaceIter f = h->face();
  FaceIter ft = t->face();

  //reassign elements: combine 2 faces
  HalfedgeIter trace = t_nxt;
  while (trace != t)
  {
    trace->face() = f;
    trace = trace->next();
  }

  //set halfedges' next correctly
  h_bf->next() = t_nxt;
  t_bf->next() = h_nxt;

  //set vertices' halfedges correctly
  v0->halfedge() = t_nxt;
  v1->halfedge() = h_nxt;

  //set face's halfedges correctly
  f->halfedge() = h_nxt;

  //delete unused elements: the erased edge, 2 of its half edges, and 1 face
  deleteEdge(e);
  deleteHalfedge(h);
  deleteHalfedge(t);
  deleteFace(ft);

  //iterate through the face before return in order to check whether
  //there is one edge hanging. if so, call eraseEdge and delete it until no hanging edges
  // exist

  HalfedgeIter htmp = f->halfedge();
  while (need_erase(htmp))
  {
      HalfedgeIter tmp = htmp;
      do{
        if (tmp->face() == tmp->twin()->face())
        {
          f = eraseEdge(tmp->edge());
          break;
         }
        tmp = tmp->next();
      }while (tmp != htmp);
      htmp = f->halfedge();
  }

  return f;
//  HalfedgeIter tmp = h_nxt;
//  do{
//    if (tmp->face() == tmp->twin()->face())
//    {
//      f = eraseEdge(tmp->edge());
//      break;
//    }
//    tmp = tmp->next();
//  }while (tmp != h_nxt);
//
}
bool HalfedgeMesh:: need_erase(HalfedgeIter h)
{
    // a helper function that helps determine if the current face contains
    // haning edge. If so, delete it
    HalfedgeIter trace = h;
    bool result = false;
    do{
        if (trace->face() == trace->twin()->face())
        {
            result =true;
            break;
        }
        trace = trace->next();
    } while(trace != h);

    return result;
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should flip the given edge and return an iterator to the
  // flipped edge.

  if (e0->isBoundary())
  {
    showError("boundary edge cannot be flipped");
    return e0;
  }

  HalfedgeIter h = e0->halfedge();
  HalfedgeIter t = h->twin();

  HalfedgeIter v0_new = t->next()->next();
  HalfedgeIter v1_new = h->next()->next();

  //flip the edge by erase the old one and add a new one
  eraseEdge(e0);
  return addEdge(v0_new, v1_new);
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Unlike the local mesh operations (like bevel or edge flip), we will perform
  // subdivision by splitting *all* faces into quads "simultaneously."  Rather
  // than operating directly on the halfedge data structure (which as you've
  // seen
  // is quite difficult to maintain!) we are going to do something a bit nicer:
  //
  //    1. Create a raw list of vertex positions and faces (rather than a full-
  //       blown halfedge mesh).
  //
  //    2. Build a new halfedge mesh from these lists, replacing the old one.
  //
  // Sometimes rebuilding a data structure from scratch is simpler (and even
  // more
  // efficient) than incrementally modifying the existing one.  These steps are
  // detailed below.

  // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here
  // we're
  // going to do something a little bit strange: since we will have one vertex
  // in
  // the subdivided mesh for each vertex, edge, and face in the original mesh,
  // we
  // can nicely store the new vertex *positions* as attributes on vertices,
  // edges,
  // and faces of the original mesh.  These positions can then be conveniently
  // copied into the new, subdivided mesh.
  // [See subroutines for actual "TODO"s]

  if (useCatmullClark) {
    //this method is for meshes without boundary only!
    for (auto &e: edges)
    {
      if (e.isBoundary()) {
        showError("mesh has boundary, can't do Carmull Clark subdivision");
        return;
      }
    }
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // and
  // face in the original mesh.  These indices will be the indices of the
  // vertices
  // in the new (subdivided mesh).  They do not have to be assigned in any
  // particular
  // order, so long as no index is shared by more than one mesh element, and the
  // total number of indices is equal to V+E+F, i.e., the total number of
  // vertices
  // plus edges plus faces in the original mesh.  Basically we just need a
  // one-to-one
  // mapping between original mesh elements and subdivided mesh vertices.
  // [See subroutine for actual "TODO"s]
  assignSubdivisionIndices();

  // TODO Step III: Build a list of quads in the new (subdivided) mesh, as
  // tuples of
  // the element indices defined above.  In other words, each new quad should be
  // of
  // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
  // original mesh elements.  Note that it is essential to get the orientation
  // right
  // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
  // circulate in the same direction as old faces (think about the right-hand
  // rule).
  // [See subroutines for actual "TODO"s]
  vector<vector<Index> > subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // TODO Step IV: Pass the list of vertices and quads to a routine that clears
  // the
  // internal data for this halfedge mesh, and builds new halfedge data from
  // scratch,
  // using the two lists.
  rebuild(subDFaces, subDVertices);
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
void HalfedgeMesh::computeLinearSubdivisionPositions() {
  // TODO For each vertex, assign Vertex::newPosition to
  // its original position, Vertex::position.
  for (auto &v: vertices)
  {
    v.newPosition = v.position;
  }

  // TODO For each edge, assign the midpoint of the two original
  // positions to Edge::newPosition.
  for (auto &e: edges)
  {
    e.newPosition = e.centroid();
  }

  // TODO For each face, assign the centroid (i.e., arithmetic mean)
  // of the original vertex positions to Face::newPosition.  Note
  // that in general, NOT all faces will be triangles!
  for (auto &f: faces)
  {
    f.newPosition = f.centroid();
  }

}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
void HalfedgeMesh::computeCatmullClarkPositions() {
  // TODO The implementation for this routine should be
  // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
  // except that the calculation of the positions themsevles is
  // slightly more involved, using the Catmull-Clark subdivision
  // rules. (These rules are outlined in the Developer Manual.)

  // TODO face
  for (auto &f: faces)
  {
    f.newPosition = f.centroid();
  }

  // TODO edges
  for (auto &e: edges)
  {
    FaceIter f0 = e.halfedge()->face();
    FaceIter f1 = e.halfedge()->twin()->face();
    Vector3D pos = (2 * e.centroid() + f0->newPosition + f1->newPosition) / 4.0f;
    e.newPosition = pos;
  }

  // TODO vertices
  for (auto &v: vertices)
  {
    Vector3D Q, R = (0.0f, 0.0f, 0.0f);
    size_t n = v.degree();

    HalfedgeIter trace = v.halfedge();
    do
    {
      Q += trace->face()->newPosition;
      R += trace->edge()->newPosition;
      trace = trace->twin()->next();
    }while (trace != v.halfedge());

    Q = Q * 1.0f / n;
    R = R * 1.0f / n;

    v.newPosition = ( Q + 2 * R + (n - 3) * v.position) / (n * 1.0f);
  }
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  // TODO Start a counter at zero; if you like, you can use the
  // "Index" type (defined in halfedgeMesh.h)
  size_t index = 0;
  // TODO Iterate over vertices, assigning values to Vertex::index
  for (auto &v: vertices)
  {
    v.index = index;
    index++;
  }

  // TODO Iterate over edges, assigning values to Edge::index
  for (auto &e: edges)
  {
    e.index = index;
    index++;
  }

  // TODO Iterate over faces, assigning values to Face::index
  for (auto &f: faces)
  {
    f.index = index;
    index++;
  }
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D>& subDVertices) {
  // TODO Resize the vertex list so that it can hold all the vertices.
  subDVertices.resize(vertices.size() + edges.size() + faces.size());

  // TODO Iterate over vertices, assigning Vertex::newPosition to the
  // appropriate location in the new vertex list.
  for (auto &v: vertices)
    subDVertices[v.index] = v.newPosition;

  // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
  // location in the new vertex list.
  for (auto &e: edges)
    subDVertices[e.index] = e.newPosition;

  // TODO Iterate over faces, assigning Face::newPosition to the appropriate
  // location in the new vertex list.
  for (auto &f: faces)
    subDVertices[f.index] = f.newPosition;
}

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index> >& subDFaces) {
  // TODO This routine is perhaps the most tricky step in the construction of
  // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
  // vertex positions).  Basically what you want to do is iterate over faces,
  // then for each for each face, append N quads to the list (where N is the
  // degree of the face).  For this routine, it may be more convenient to simply
  // append quads to the end of the list (rather than allocating it ahead of
  // time), though YMMV.  You can of course iterate around a face by starting
  // with its first halfedge and following the "next" pointer until you get
  // back to the beginning.  The tricky part is making sure you grab the right
  // indices in the right order---remember that there are indices on vertices,
  // edges, AND faces of the original mesh.  All of these should get used.  Also
  // remember that you must have FOUR indices per face, since you are making a
  // QUAD mesh!

  // TODO iterate over faces
  // TODO loop around face
  // TODO build lists of four indices for each sub-quad
  // TODO append each list of four indices to face list

  for (auto &f: faces)
  {
    HalfedgeIter h = f.halfedge();
    do
    {
      vector<Index> quad( 4 );
      quad[0] = h->edge()->index;
      quad[1] = h->twin()->vertex()->index;
      quad[2] = h->next()->edge()->index;
      quad[3] = f.index;

      subDFaces.push_back(quad);
      h = h->next();

    } while(h != f.halfedge());
  }
}

vector<HalfedgeIter> HalfedgeMesh::findHalfedge(FaceIter f, vector<VertexIter> vs) {

  //this is a helper function that helps to find 2  halfedges that belong to face f
  // and their vertices are listed in vs
  HalfedgeIter h0 = f-> halfedge();
  HalfedgeIter h;
  HalfedgeIter h1 = f-> halfedge();
  bool find2 = false;
  for (int i = 0; i < f->degree();i++)
  {
    for (int j = 0; j < vs.size();j++)
    {

      if (h0->vertex() == vs[j])
      {
        h = h0;
      }
    }
    h0 = h0->next();
  }

  for (int i = 0; i < f->degree(); i++)
  {
    for (int j = 0; j < vs.size();j++)
    {
      if (!find2 && h1->vertex() == vs[j] && h1->vertex() != h0->vertex())
      {
        find2 = true;
      }
    }
    if (!find2) h1 = h1->next();
  }

  vector<HalfedgeIter> hs;
  hs.push_back(h);
  hs.push_back(h1);

  return hs;
}

FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
  // TODO This method should replace the vertex v with a face, corresponding to
  // a bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
  // implement!)

  // find all v's adjacent halfedges and add a vertex on each edge
  vector<HalfedgeIter> Adj;
  vector<VertexIter> adjV;
  HalfedgeIter h = v->halfedge();
  HalfedgeIter t = h->twin();
  HalfedgeIter trace = h;
  do
  {
    Adj.push_back(trace);
    trace = trace->twin()->next();
  }while(trace != h);

  for (auto adj: Adj)
  {
    VertexIter newadj = addVertexOnEdge(adj->edge());
    adjV.push_back(newadj);
  }

  //connect 2 newly inserted vertices in pairs
  vector<FaceIter> fs;
  HalfedgeIter transverse = v->halfedge();
  do{

    fs.push_back(transverse->face());
    transverse = transverse->twin()->next();
  }while (transverse != v->halfedge());

  for (int i = 0; i < fs.size(); i++)
  {

    vector<HalfedgeIter> hs =  findHalfedge(fs[i], adjV);
    addEdge(hs[0], hs[1]);
  }
  //finally erase the original vertex in the middle
  return eraseVertex(v);
}

FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
  // TODO This method should replace the edge e with a face, corresponding to a
  // bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
  // implement!)

  showError("bevelEdge() not implemented.");
  return facesBegin();
}
VertexIter HalfedgeMesh::addVertexOnEdge(EdgeIter e)
{
  //a helper function that adds a vertex on edge and split it into 2 edges

  VertexIter v = newVertex();
  HalfedgeIter a = newHalfedge();
  HalfedgeIter b = newHalfedge();
  EdgeIter e0 = newEdge();
  v->position = e->centroid();
  v->isNew = true;
  v->halfedge() = a;
  a->vertex() = v;
  b->vertex() = v;

  HalfedgeIter h = e->halfedge();
  HalfedgeIter t = h->twin();
  HalfedgeIter h_nxt = h->next();
  HalfedgeIter t_nxt = t->next();

  FaceIter f0 = h->face();
  FaceIter f1 = t->face();
  h->next() = a;
  a->next() = h_nxt;
  t->next() = b;
  b->next() = t_nxt;
  a->twin() = t;
  t->twin() = a;
  b->twin() = h;
  h->twin() = b;

  t->edge() = e0;
  a->edge() = e0;
  b->edge() = e;
  e->halfedge() = h;
  e0->halfedge() = a;

  a->face() = f0;
  b->face() = f1;

  v->halfedge()->edge()->isNew = false;
  v->halfedge()->twin()->next()->edge()->isNew = false;
  return v;
}


FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
  // TODO This method should replace the face f with an additional, inset face
  // (and ring of faces around it), corresponding to a bevel operation. It
  // should return the new face.  NOTE: This method is responsible for updating
  // the *connectivity* of the mesh only---it does not need to update the vertex
  // positions.  These positions will be updated in
  // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
  // implement!)
  vector <HalfedgeIter> hs;
  HalfedgeIter h = f->halfedge();
  do
  {
    hs.push_back(h);
    h = h->next();
  }while (h!= f->halfedge());

  vector<VertexIter> vs; // new vertices
  vector<HalfedgeIter> new_hs; // halfedges that connects the new vertices to original vertices
  vector<HalfedgeIter> bevelhs;// halfedges of the beveled face
  for (int i = 0; i < hs.size(); i++)
  {
      VertexIter v = newVertex();
      HalfedgeIter newh = newHalfedge();
      HalfedgeIter newh_t = newHalfedge();
      EdgeIter e = newEdge();

      e->halfedge() = newh;
      newh->edge() = e;
      newh_t->edge() = e;

      v->position = hs[i]->vertex()->position;
      v->halfedge() = newh;
      newh->vertex() = v;
      newh_t->vertex() = hs[i]->vertex();

      newh->twin() = newh_t;
      newh_t->twin() = newh;

      newh->face() = f;
      newh_t->face() = f;
      newh_t->next() = newh;

      vs.push_back(v);
      new_hs.push_back(newh);
  }

  if (vs.size() != new_hs.size())
      showError("size different : error");

  for (int i = 0; i < vs.size(); i++)
  {

      HalfedgeIter connect = newHalfedge();
      HalfedgeIter connect_t = newHalfedge();
      EdgeIter e = newEdge();
      FaceIter fn = newFace();

      e->halfedge() = connect;
      connect->edge() = e;
      connect_t->edge() = e;

      connect->twin() = connect_t;
      connect_t->twin() = connect;

      connect->vertex() = vs[(i+1)%vs.size()];
      connect_t->vertex() = vs[i];

      new_hs[i]->next() = hs[i];
      hs[i]->next() = new_hs[(i+1)%new_hs.size()]->twin();
      connect->next() = new_hs[i];
      new_hs[(i+1)%new_hs.size()]->twin()->next() = connect;

      fn->halfedge() = new_hs[i];
      HalfedgeIter trace = fn->halfedge();
      do{
          trace->face() = fn;
          trace = trace->next();
      }while(trace != fn->halfedge());

      bevelhs.push_back(connect_t);
  }

  for (int i = 0; i < bevelhs.size(); i++)
  {
      bevelhs[i]->next() = bevelhs[(i+1)% bevelhs.size()];
      bevelhs[i]->face() = f;
  }

  f->halfedge() = bevelhs[0];
  return f;
}


void HalfedgeMesh::bevelFaceComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double normalShift,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled face.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the originalVertexPositions array) to compute an offset vertex
  // position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); hs++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

  //the position of the vertices will be located at middle by default (click only, no drag);
  //if (normalShift == 0 && tangentialInset == 0) return;
  if (newHalfedges.size() != originalVertexPositions.size()) return;

  //find the normal verctor by crossing 2 vector of the original mesh, normalize it, and times normalShift amount
  Vector3D norm = (0,0,0);
  HalfedgeIter h = newHalfedges[0];
  Vector3D p0 = h->next()->next()->twin()->next()->next()->vertex()->position;
  Vector3D p1 = h->twin()->vertex()->position;
  Vector3D p2 = h->next()->next()->vertex()->position;
  Vector3D V0_2 = p2 - p0;
  Vector3D V1_2 = p2 - p1;
  norm = cross(V1_2, V0_2);
  norm.normalize();
  norm = norm * normalShift;

  //find the center by taking average of the coordinates of the vertices
  Vector3D center = (0,0,0);
  for (int i = 0; i < originalVertexPositions.size(); i++)
  {
    center += originalVertexPositions[i];
  }
  center = center * 1.0f / originalVertexPositions.size();

  //compute tangential shift and apply new position
  for (int i = 0; i < newHalfedges.size(); i++)
  {
    Vector3D tangent = (originalVertexPositions[i] - center) * tangentialInset;
    newHalfedges[i]->vertex()->position += tangent + norm;
  }

}

void HalfedgeMesh::bevelVertexComputeNewPositions(
    Vector3D originalVertexPosition, vector<HalfedgeIter>& newHalfedges,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled vertex.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., hs.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.

//  for (auto &h : newHalfedges)
//    std::cout << h->vertex()->position << std::endl;

  for (size_t i = 0; i < newHalfedges.size(); i++)
  {
    Vector3D lim1 = newHalfedges[i]->vertex()->position - originalVertexPosition;
    Vector3D tangent = lim1 * tangentialInset;
    Vector3D lim2 = newHalfedges[i]->twin()->vertex()->position - newHalfedges[i]->vertex()->position;
    //check upper and lower bounds on how big/small bevel vertex can have
    if (tangent.norm() >= lim1.norm() && tangentialInset < 0) return;
    else if (tangent.norm() >= lim2.norm() && tangentialInset > 0) return;
    newHalfedges[i]->vertex()->position += tangent;
  }
}

void HalfedgeMesh::bevelEdgeComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled edge.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); i++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::splitPolygons(vector<FaceIter>& fcs) {
  for (auto f : fcs) splitPolygon(f);
}

void HalfedgeMesh::splitPolygon(FaceIter f) {
  // this funnction splits the given face of degree >=3 into faces of degree 3 only
  if(f->degree() == 3) return;

  HalfedgeIter h = f->halfedge();
  HalfedgeIter h1 = h->next();
  HalfedgeIter h0 = h1;
  while (h0->next() != h) h0 = h0->next();

  //make one triangle face by adding on edge
  EdgeIter e = addEdge(h0, h1);
  //if the face remaining is still not a triangle, split again
  if (e->halfedge()->face()->degree() == 3)
      splitPolygon(e->halfedge()->twin()->face());
  else
      splitPolygon(e->halfedge()->face());
}

EdgeRecord::EdgeRecord(EdgeIter& _edge) : edge(_edge) {
  // TODO: (meshEdit)
  // Compute the combined quadric from the edge endpoints.
  // -> Build the 3x3 linear system whose solution minimizes the quadric error
  //    associated with these two endpoints.
  // -> Use this system to solve for the optimal position, and store it in
  //    EdgeRecord::optimalPoint.
  // -> Also store the cost associated with collapsing this edg in
  //    EdgeRecord::Cost.
}

void MeshResampler::upsample(HalfedgeMesh& mesh)
// This routine should increase the number of triangles in the mesh using Loop
// subdivision.
{
  //this method is for triangle meshes without boundary only
  for (EdgeIter e = mesh.edgesBegin(); e !=mesh.edgesEnd(); e++) {
    if (e->isBoundary()) {
      showError("mesh has boundary, can't do loop subdivision");
      return;
    }

    if (e->halfedge()->face()->degree() != 3) {
      showError("not triangle mesh, can't do loop subdivision");
      return;
    }
  }

  // TODO: (meshEdit)
  // Compute new positions for all the vertices in the input mesh, using
  // the Loop subdivision rule, and store them in Vertex::newPosition.
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
  {
    HalfedgeIter h = v->halfedge();
    Vector3D adjPos = (0.0f, 0.0f, 0.0f);
    size_t n = 0;
    float u;
    do {
      adjPos += h->twin()->vertex()->position;
      h = h->twin()->next();
      n++;
    } while(h != v->halfedge());

    if (n == 3) u = 3.0f / 16;
    else u = 3.0f / ( 8.0 * n );
    v->newPosition = u * adjPos + (1.0 - n * u) * v->position;
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh.
    v->isNew = false;
  }

  // -> Next, compute the updated vertex positions associated with edges, and
  //    store it in Edge::newPosition.
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
  {
    HalfedgeIter h = e->halfedge();
    HalfedgeIter t = h->twin();
    Vector3D ABpos = h->vertex()->position + t->vertex()->position;
    Vector3D CDpos = h->next()->next()->vertex()->position + t->next()->next()->vertex()->position;
    e->newPosition = ( 3.0f * ABpos + CDpos ) / 8.0f;
    e->isNew = false;
  }


  // -> Next, we're going to split every edge in the mesh, in any order.  For
  //    future reference, we're also going to store some information about which
  //    subdivided edges come from splitting an edge in the original mesh, and
  //    which edges are new, by setting the flat Edge::isNew. Note that in this
  //    loop, we only want to iterate over edges of the original mesh.
  //    Otherwise, we'll end up splitting edges that we just split (and the
  //    loop will never end!)
  size_t n = mesh.nEdges();
  EdgeIter e = mesh.edgesBegin();
  for (int i = 0; i < n; i++)
  {
    while (e->isNew) e++;
    EdgeIter nextEdge = e;
    Vector3D pos = e->newPosition;
    nextEdge++;


    VertexIter v = mesh.splitEdge(e);
    v->newPosition = pos;
    v->isNew = true;
    e = nextEdge;
  }

  // -> Now flip any new edge that connects an old and new vertex.
  vector<EdgeIter> edges;
  for (EdgeIter track = mesh.edgesBegin(); track != mesh.edgesEnd(); track++)
  {
    if (track->isNew &&
       (track->halfedge()->vertex()->isNew != track->halfedge()->twin()->vertex()->isNew))
       edges.push_back(track);
  }

  for (int j = 0; j < edges.size(); j++) {
    EdgeIter e0 = edges[j];
    if ((e0->halfedge()->vertex()->isNew != e0->halfedge()->twin()->vertex()->isNew)
        && e0->isNew) {
      EdgeIter e0_flipped = mesh.flipEdge(e0);
      e0_flipped->isNew = true;
    }
  }

  // -> Finally, copy the new vertex positions into final Vertex::position.
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
  {
    v->position = v->newPosition;
  }


  // Each vertex and edge of the original surface can be associated with a
  // vertex in the new (subdivided) surface.
  // Therefore, our strategy for computing the subdivided vertex locations is to
  // *first* compute the new positions
  // using the connectity of the original (coarse) mesh; navigating this mesh
  // will be much easier than navigating
  // the new subdivided (fine) mesh, which has more elements to traverse.  We
  // will then assign vertex positions in
  // the new mesh based on the values we computed for the original mesh.

  // Compute updated positions for all the vertices in the original mesh, using
  // the Loop subdivision rule.

  // Next, compute the updated vertex positions associated with edges.

  // Next, we're going to split every edge in the mesh, in any order.  For
  // future
  // reference, we're also going to store some information about which
  // subdivided
  // edges come from splitting an edge in the original mesh, and which edges are
  // new.
  // In this loop, we only want to iterate over edges of the original
  // mesh---otherwise,
  // we'll end up splitting edges that we just split (and the loop will never
  // end!)

  // Finally, flip any new edge that connects an old and new vertex.

  // Copy the updated vertex positions to the subdivided mesh.
}

void MeshResampler::downsample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in Face::quadric
  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in Vertex::quadric
  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an EdgeRecord for each edge and sticking it in the
  //    queue.
  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.
  showError("downsample() not implemented.");
}

void MeshResampler::resample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute the mean edge length.
  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  showError("resample() not implemented.");
}

}  // namespace CMU462
