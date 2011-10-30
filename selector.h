#ifndef SELECTOR_H
#define SELECTOR_H

#include <Eigen/Core>
#include "mesh.h"

class Camera;
class Controller;

class Selector
{
public:
    Selector(Camera &c);

    enum SelectMode {SM_DRAG, SM_SETANCHOR, SM_CLEARANCHOR, SM_DELETEFACE};

    void setController(Controller &c);

    void startEditing(const Eigen::Vector2d &pos, SelectMode mode);
    void primitiveSelected(Mesh::PrimType primtype, int primidx);
    void updateEditing(const Eigen::Vector2d &pos);
    void stopEditing();

    bool selectionQueued() {return state_ == SS_PICKQUEUED;}
    Eigen::Vector2d getQueuedPos();

private:
    enum SelectorState {SS_NONE, SS_PICKQUEUED, SS_DRAGGING};


    Camera &c_;
    Controller *cont_;
    SelectorState state_;
    SelectMode mode_;
    Eigen::Vector2d startPos_;
    Mesh::PrimType dragtype_;
    int dragidx_;
};

#endif // SELECTOR_H
