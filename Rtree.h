#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cassert>

typedef unsigned int  uint;
typedef unsigned char uchar;

class Point {
public:
    float x, y;
    Point() : x(0.0f), y(0.0f) {}
    Point(float x, float y) : x(x), y(y) {}

    // Distancia euclidiana
    float distanceTo(const Point &other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

class RNode;

class MBB {
public:
    Point lower; // Esquina inferior izquierda
    Point upper; // Esquina superior derecha

    MBB() : lower(Point()), upper(Point()) {}
    MBB(const Point &p1, const Point &p2)
        : lower(Point(std::min(p1.x, p2.x), std::min(p1.y, p2.y))),
          upper(Point(std::max(p1.x, p2.x), std::max(p1.y, p2.y))) {}

    float area() const {
        return (upper.x - lower.x) * (upper.y - lower.y);
    }
    
    float semiPerimeter() const {
        // Debug: No es 2*(lo_de_abajo) porque el semiperimetro es la mitad del perimetro.
        return ((upper.x - lower.x) + (upper.y - lower.y));
    }

    float distanceTo(const Point &p) const {
        // Depende de si el punto está dentro o fuera del MBB.
        float dx = std::max({0.0f, lower.x - p.x, p.x - upper.x});
        float dy = std::max({0.0f, lower.y - p.y, p.y - upper.y});
        return std::sqrt(dx * dx + dy * dy);   

    }

    // Incremento del semiperimetro si agrega p
    float deltaSemiPerimeter(const Point &p) const {
        MBB aux = *this;
        aux.expandToInclude(p);
        return aux.semiPerimeter();
    }

    // Expande para incluir point o MBB
    void expandToInclude(const Point &p){
        lower.x = std::min(lower.x, p.x);
        lower.y = std::min(lower.y, p.y);
        upper.x = std::max(upper.x, p.x);
        upper.y = std::max(upper.y, p.y);
    }
    void expandToInclude(const MBB &other){
        lower.x = std::min(lower.x, other.lower.x);
        lower.y = std::min(lower.y, other.lower.y);
        upper.x = std::max(upper.x, other.upper.x);
        upper.y = std::max(upper.y, other.upper.y);
    }

    // Retorna el area de interseccion
    float intersects(const MBB &other) const{
        
        // No hay intersección en x.
        if (upper.x < other.lower.x || lower.x > other.upper.x) return 0;
        
        // No hay intersección en y.
        if (upper.y < other.lower.y || lower.y > other.upper.y) return 0;

        // Hay intersección.
        return 1;  
    }
    
    
    // Crear MBB a partir de un vector de puntos
    // EDITAR.
    static MBB computeFromPoints(const std::vector<Point> &pts) {
        // Si no hay puntos.
        if (pts.empty()) {
            return MBB();
        }
    
        // Si hay puntos.
        MBB aux(pts[0], pts[0]);

        for (const Point &pt : pts) {
            aux.expandToInclude(pt);
        }
    
        return aux;
    }

    // Crear MBB a partir de un vector de nodos
    static MBB computeFromNodes(const std::vector<RNode*> &nodes);

    // Union MBBs
    static MBB unionOf(const MBB &a, const MBB &b);
};


// -------------------------------
// Clase RNode
// -------------------------------
class RNode {
private:
    // Linear Split para nodos hojas
    // Ref: Session 3.1 - pág 24.
    RNode* linearSplitLeaf(uchar maxEntries){
        size_t ind_p1 = 0;
        size_t ind_p2 = 1;
        size_t min_capa = (maxEntries+1) / 2;
        float max_dist = points[0].distanceTo(points[1]);
        
        // Buscamos los dos puntos más alejados.
        for (size_t i = 0; i < points.size(); i++)
            for (size_t j=i+1; j < points.size(); j++)
                if (points[i].distanceTo(points[j]) > max_dist) {
                    ind_p1 = i;
                    ind_p2 = j;
                    max_dist = points[i].distanceTo(points[j]);
                }


                
        RNode* n1 = new RNode(true);
        RNode* n2 = new RNode(true);
        n1->points.push_back(points[ind_p1]);
        n2->points.push_back(points[ind_p2]);
        
        // Agregamos los puntos que quedan a n1 o n2.
        for (size_t i = 0; i < points.size(); i++) {
            if (i == ind_p1 || i == ind_p2)
                continue;

            // Lidiando con nodos llenos.
            if (n1->points.size() >= maxEntries + 1 - min_capa) {
                n2->points.push_back(points[i]);
                n2->mbr.expandToInclude(points[i]);
            } else if (n2->points.size() >= maxEntries + 1 - min_capa) {
                n1->points.push_back(points[i]);
                n1->mbr.expandToInclude(points[i]);
            }

            // Nodonolleno.
            else {
                if (MBB::computeFromPoints(n1->points).deltaSemiPerimeter(points[i]) < MBB::computeFromPoints(n2->points).deltaSemiPerimeter(points[i])) {
                    n1->points.push_back(points[i]);
                    n1->mbr.expandToInclude(points[i]);
                } else {
                    n2->points.push_back(points[i]);
                    n2->mbr.expandToInclude(points[i]);
                }
            }
        }

        // Debug: MBBS actualiseishon.
        n1->mbr = MBB::computeFromPoints(n1->points);
        n2->mbr = MBB::computeFromPoints(n2->points);

        *this = *n1;
        return n2;
    }

    // Quadratic Split para nodos internos
    // Ref: Session 3.1 - pág 24.
    RNode* quadraticSplitInternal(uchar maxEntries){
        size_t ind_p1 = 0;
        size_t ind_p2 = 1;
        size_t min_capa = (maxEntries+1) / 2;
        float dif_max = -1;

        // Buscando los nodos con mayor espacio muerto.
        for (size_t i = 0; i < children.size(); i++)
            for (size_t j = i + 1; j < children.size(); j++) {
                float dif = children[i]->mbr.area() + children[j]->mbr.area();
                if (dif > dif_max) {
                    ind_p1 = i;
                    ind_p2 = j;
                    dif_max = dif;
                }
            }

        

        RNode* n1 = new RNode(false);
        RNode* n2 = new RNode(false);
        n1->children.push_back(children[ind_p1]);
        n2->children.push_back(children[ind_p2]);

        for (size_t i = 0; i < children.size(); i++) {
            if (i == ind_p1 || i == ind_p2)
                continue;
            
            // Copipaste de arriba.
            if (n1->children.size() >=  ((maxEntries + 1) - min_capa) ) {
                n2->children.push_back(children[i]);
                n2->mbr.expandToInclude(children[i]->mbr);
            } else if (n2->children.size() >=  ((maxEntries + 1) - min_capa) ) {
                n1->children.push_back(children[i]);
                n1->mbr.expandToInclude(children[i]->mbr);

            } else {
                if (MBB::computeFromNodes(n1->children).deltaSemiPerimeter(children[i]->mbr.lower) <
                    MBB::computeFromNodes(n2->children).deltaSemiPerimeter(children[i]->mbr.lower)) {
                    n1->children.push_back(children[i]);
                    n1->mbr.expandToInclude(children[i]->mbr);
                } else {
                    n2->children.push_back(children[i]);
                    n2->mbr.expandToInclude(children[i]->mbr);
                }
            }
        }

        // Debug.
        n1->mbr = MBB::computeFromNodes(n1->children);
        n2->mbr = MBB::computeFromNodes(n2->children);

        *this = *n1;
        return n2;
    }

public:
    bool isLeaf;
    MBB mbr;
    std::vector<Point>    points;
    std::vector<RNode*> children;

    RNode(bool leaf) : isLeaf(leaf) {}

    // Session 3.1 - pág 21.
    RNode* insert(const Point &p, uchar maxEntries){
        // linea del 1 al 4.
        if (isLeaf) {
            points.push_back(p);
            mbr.expandToInclude(p);
            if (points.size() > maxEntries) {
                return linearSplitLeaf(maxEntries);
            }
        
        // lineas del 5 al 7
        } else {
            RNode* posible_kid = nullptr;
            float posible_incr = std::numeric_limits<float>::infinity();
            
            for (RNode* child : children) {
                float incr = child->mbr.deltaSemiPerimeter(p);
                if (incr < posible_incr) {
                    posible_incr = incr;
                    posible_kid = child;
                }
            }
            


            RNode* aux = posible_kid->insert(p, maxEntries);
            
            if (aux) {
                children.push_back(aux);
                mbr.expandToInclude(aux->mbr);

                if (children.size() > maxEntries){
                    return quadraticSplitInternal(maxEntries);
                }
                    
            } else {
                mbr.expandToInclude(posible_kid->mbr);
            }
        }
        return nullptr;
    }




    std::vector<Point> search(const MBB &query) const {
        std::vector<Point> aux;

        // Sin intersecciones.
        if (!mbr.intersects(query)){
            return aux;
        }
        
        if (isLeaf) {
            for (const Point &pt : points) {
                if ((pt.x >= query.lower.x) && (pt.x <= query.upper.x) && (pt.y >= query.lower.y) && (pt.y <= query.upper.y)) {
                    aux.push_back(pt);
                }
            }
        
        // Nodo interno.
        } else {
            for (const RNode* child : children) {
                std::vector<Point> aux_hijos = child->search(query);
                aux.insert(aux.end(), aux_hijos.begin(), aux_hijos.end());
            }
        }
        return aux;
    }

};



// -------------------------------
// Para Best-First
// -------------------------------
struct QueueEntry {
    float distance;  // Distancia desde el query al MBB
    bool isNode;     // Si true, es un nodo; si false, es un punto
    RNode* node;
    Point pt;
};

struct QueueEntryComparator {
    bool operator()(const QueueEntry &a, const QueueEntry &b) const {
        return a.distance > b.distance;
    }
};

// -------------------------------
// Clase RTree
// -------------------------------
class RTree {
public:
    RNode* root;
    uchar maxEntries;  // Capacidad maxima

    RTree(uchar maxEntries = 3) : maxEntries(maxEntries) {
        root = new RNode(true);
    }

    void insert(const Point &p){
        RNode* new_n = root->insert(p, maxEntries);

        if (new_n) {
            RNode* new_root = new RNode(false);
            
            new_root->children.push_back(root);
            new_root->children.push_back(new_n);
            new_root->mbr = root->mbr;
            new_root->mbr.expandToInclude(new_n->mbr);

            root = new_root;
        }
    }
    


    std::vector<Point> search(const MBB &query) const {
        return root ? root->search(query) : std::vector<Point>();
    }



    std::vector<Point> kNN(const Point &query, uchar k) const {
        std::vector<Point> rpta;

        std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryComparator> pq;
        pq.push({root->mbr.distanceTo(query), true, root, Point()});
        
        while (!pq.empty() && rpta.size() < k) {
            QueueEntry entry = pq.top(); pq.pop();

            if (entry.isNode) {
                RNode* node = entry.node;

                if (node->isLeaf) {
                    for (const Point& pt : node->points) {
                        float dist = pt.distanceTo(query);
                        pq.push({dist, false, nullptr, pt});
                    }

                // Nodo interno.
                } else {
                    for (RNode* child : node->children) {
                        pq.push({child->mbr.distanceTo(query), true, child, Point()});
                    }
                }

            // Solo pusheamos los puntos que están dentro del k.
            } else {
                rpta.push_back(entry.pt);
            }
        }
        return rpta;
    }
};


// Debug: Si no lo ponía al final no corría adfasfakl profe trol con el orden

MBB MBB::computeFromNodes(const std::vector<RNode*> &nodes) {
    if (nodes.empty()){
        return MBB();
    }

    MBB aux = nodes[0]->mbr;
    for (size_t i = 1; i < nodes.size(); ++i) {
        aux.expandToInclude(nodes[i]->mbr);
    }
    return aux;
}