#include "selector.h"
#include "camera.h"
#include "controller.h"
#include <iostream>
#include "mesh.h"

using namespace std;

using namespace Eigen;

Selector::Selector(Camera &c) : c_(c), cont_(NULL), state_(SS_NONE), mode_(SM_DRAGFREE)
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
        case SM_DRAGHEIGHT:
        case SM_DRAGFREE:
        case SM_DRAGTOP:
        {
            if(primtype != Mesh::PT_VERTEX)
                state_ = SS_RECTANGLE;
            else
            {
                state_ = SS_DRAGGING;
                vector<int> tohandle;
                tohandle.push_back(primidx);
                cont_->setHandle(tohandle);
            }
            break;
        }
        case SM_CLEARHANDLE:
        {
            if(primtype != Mesh::PT_VERTEX)
                state_ = SS_RECTANGLE;
            else
            {
                state_ = SS_NONE;
                vector<int> tounhandle;
                tounhandle.push_back(primidx);
                cont_->clearHandle(tounhandle);
            }
            break;
        }
        case SM_ADDPIN:
        {
            if(primtype != Mesh::PT_VERTEX)
                state_ = SS_RECTANGLE;
            else
            {
                state_ = SS_NONE;
                vector<int> topin;
                topin.push_back(primidx);
                cont_->setPin(topin);
            }
            break;
        }
        case SM_DELETEPIN:
        {
            if(primtype != Mesh::PT_VERTEX)
                state_ = SS_RECTANGLE;
            else
            {
                state_ = SS_NONE;
                vector<int> tounpin;
                tounpin.push_back(primidx);
                cont_->clearPin(tounpin);
            }
            break;
        }
        case SM_ADDANCHOR:
        {
            if(primtype != Mesh::PT_VERTEX)
                state_ = SS_RECTANGLE;
            else
            {
                state_ = SS_NONE;
                vector<int> toanchor;
                toanchor.push_back(primidx);
                cont_->setAnchor(toanchor);
            }
            break;
        }
        case SM_DELETEANCHOR:
        {
            if(primtype != Mesh::PT_VERTEX)
                state_ = SS_RECTANGLE;
            else
            {
                state_ = SS_NONE;
                vector<int> tounanchor;
                tounanchor.push_back(primidx);
                cont_->clearAnchor(tounanchor);
            }
            break;
        }
        case SM_DELETEFACE:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_FACE)
                cont_->deleteFace(primidx);
            break;
        }
        default:
            assert(!"Bad select mode");
    }
}

void Selector::updateEditing(const Vector2d &pos)
{
    if(state_ == SS_DRAGGING)
    {

        Vector2d translation2D = pos-startPos_;
        Vector3d right, up, center;
        c_.getSpanningSet(right, up, center);

        Vector3d translation3D = translation2D[0]*right + translation2D[1]*up;
        if(mode_ == SM_DRAGFREE && dragtype_ == Mesh::PT_VERTEX)
            cont_->dragVertex(dragidx_, translation3D);
        else if(mode_ == SM_DRAGHEIGHT && dragtype_ == Mesh::PT_VERTEX)
            cont_->dragVertexHeight(dragidx_, translation3D);
        else if(mode_ == SM_DRAGTOP && dragtype_ == Mesh::PT_VERTEX)
            cont_->dragVertexTop(dragidx_, translation3D);

        startPos_ = pos;
    }
}

void Selector::stopEditing(const Vector2d &pos)
{
    if(state_ == SS_RECTANGLE)
    {
        vector<int> selected = cont_->selectRectangle(startPos_, pos, c_);
        switch(mode_)
        {
            case SM_DRAGFREE:
            case SM_DRAGHEIGHT:
            case SM_DRAGTOP:
            {
                cont_->setHandle(selected);
                break;
            }
            case SM_ADDPIN:
            {
                cont_->setPin(selected);
                break;
            }
            case SM_DELETEPIN:
            {
                cont_->clearPin(selected);
                break;
            }
            case SM_ADDANCHOR:
            {
                cont_->setAnchor(selected);
                break;
            }
            case SM_DELETEANCHOR:
            {
                cont_->clearAnchor(selected);
                break;
            }
            case SM_CLEARHANDLE:
            {
                cont_->clearHandle(selected);
                break;
            }
            default:
                break;
        }

    }
    state_ = SS_NONE;
}

Vector2d Selector::getQueuedPos()
{
    assert(selectionQueued());
    return startPos_;
}
