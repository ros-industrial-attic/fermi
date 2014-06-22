#include <QStringList>

 #include <moveit_cartesian_planner/point_tree_item.h>

PoinTtreeItem::PoinTtreeItem(const QVector<QVariant> &data, PoinTtreeItem *parent)
{
     parentItem = parent;
     itemData = data;
}

PoinTtreeItem::~PoinTtreeItem()
{
     qDeleteAll(childItems);
}

PoinTtreeItem *PoinTtreeItem::child(int number)
{
     return childItems.value(number);
}

int PoinTtreeItem::childCount() const
{
     return childItems.count();
}

int PoinTtreeItem::childNumber() const
{
     if (parentItem)
         return parentItem->childItems.indexOf(const_cast<PoinTtreeItem*>(this));

     return 0;
}

int PoinTtreeItem::columnCount() const
{
     return itemData.count();
}

QVariant PoinTtreeItem::data(int column) const
{
     return itemData.value(column);
}

bool PoinTtreeItem::insertChildren(int position, int count, int columns)
{
     if (position < 0 || position > childItems.size())
         return false;

     for (int row = 0; row < count; ++row) {
         QVector<QVariant> data(columns);
         PoinTtreeItem *item = new PoinTtreeItem(data, this);
         childItems.insert(position, item);
     }

     return true;
}

bool PoinTtreeItem::insertColumns(int position, int columns)
{
     if (position < 0 || position > itemData.size())
         return false;

     for (int column = 0; column < columns; ++column)
         itemData.insert(position, QVariant());

     PoinTtreeItem *child;
     for(int i=0;i<childItems.count();i++)
     {

        child = childItems.at(i);
        child->insertColumns(position,columns);
     }

     return true;
}

PoinTtreeItem *PoinTtreeItem::parent()
{
     return parentItem;
}

bool PoinTtreeItem::removeChildren(int position, int count)
{
     if (position < 0 || position + count > childItems.size())
         return false;

     for (int row = 0; row < count; ++row)
         delete childItems.takeAt(position);

     return true;
}

bool PoinTtreeItem::removeColumns(int position, int columns)
{
     if (position < 0 || position + columns > itemData.size())
         return false;

     for (int column = 0; column < columns; ++column)
         itemData.remove(position);

          PoinTtreeItem *child;
     for(int i=0;i<childItems.count();i++)
     {

        child = childItems.at(i);
        child->removeColumns(position,columns);
     }

     return true;
}

bool PoinTtreeItem::setData(int column, const QVariant &value)
{
     if (column < 0 || column >= itemData.size())
         return false;

     itemData[column] = value;
     return true;
}