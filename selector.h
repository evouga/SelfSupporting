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

    enum SelectMode {SM_DRAGFREE, SM_DRAGHEIGHT, SM_CLEARHANDLE, SM_DELETEFACE, SM_ADDPIN, SM_DELETEPIN, SM_DRAGTOP, SM_ADDANCHOR, SM_DELETEANCHOR};

    void setController(Controller &c);

    void startEditing(const Eigen::Vector2d &pos, SelectMode mode);
    void primitiveSelected(Mesh::PrimType primtype, int primidx);
    void updateEditing(const Eigen::Vector2d &pos);
    void stopEditing(const Eigen::Vector2d &pos);

    bool selectionQueued() {return state_ == SS_PICKQUEUED;}
    Eigen::Vector2d getQueuedPos();

private:
    enum SelectorState {SS_NONE, SS_PICKQUEUED, SS_DRAGGING, SS_RECTANGLE};


    Camera &c_;
    Controller *cont_;
    SelectorState state_;
    SelectMode mode_;
    Eigen::Vector2d startPos_;
    Mesh::PrimType dragtype_;
    int dragidx_;
};

#endif // SELECTOR_H
