#include "selector.h"
#include "camera.h"
#include "controller.h"
#include <iostream>
#include "mesh.h"

using namespace std;

using namespace Eigen;

Selector::Selector(Camera &c) : c_(c), cont_(NULL), state_(SS_NONE), mode_(SM_DRAG)
{
    startPos_.setZero();
}

void Selector::setController(Controller &c)
{
    cont_ = &c;
}

void Selector::startEditing(const Vector2d &pos, SelectMode mode)
{
    state_ = SS_PICKQUEUED;
    mode_ = mode;
    startPos_ = pos;
}

void Selector::primitiveSelected(Mesh::PrimType primtype, int primidx)
{
    dragtype_ = primtype;
    dragidx_ = primidx;
    switch(mode_)
    {
        case SM_DRAG:
        {
            state_ = SS_DRAGGING;
            if(primtype == Mesh::PT_VERTEX)
            {
                cont_->setAnchor(primidx);
            }
            break;
        }
        case SM_SETANCHOR:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_VERTEX)
            {
                cont_->setAnchor(primidx);
            }
            break;
        }
        case SM_CLEARANCHOR:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_VERTEX)
            {
                cont_->clearAnchor(primidx);
            }
            break;
        }
        default:
            assert(!"Bad select mode");
    }
}

void Selector::updateEditing(const Vector2d &pos)
{
    if(state_ != SS_DRAGGING)
        return;

    Vector2d translation2D = pos-startPos_;
    Vector3d right, up, center;
    c_.getSpanningSet(right, up, center);

    Vector3d translation3D = translation2D[0]*right + translation2D[1]*up;
//    if(dragtype_ == Mesh::PT_FACE)
//        cont_->translateFace(dragidx_, translation3D);
    if(dragtype_ == Mesh::PT_VERTEX)
        cont_->dragVertex(dragidx_, translation3D);

    startPos_ = pos;
}

void Selector::stopEditing()
{
    state_ = SS_NONE;
}

Vector2d Selector::getQueuedPos()
{
    assert(selectionQueued());
    return startPos_;
}
