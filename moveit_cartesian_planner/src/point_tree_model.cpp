#include <QtGui>

#include <moveit_cartesian_planner/point_tree_item.h>
#include <moveit_cartesian_planner/point_tree_model.h>

PoinTtreeModel::PoinTtreeModel(const QStringList &headers, const QString &data,
                     QObject *parent)
    : QAbstractItemModel(parent)
{
    QVector<QVariant> rootData;
    //QString header;

    for(int i=0;i<headers.count();i++)
         rootData << headers.at(i);

    rootItem = new PoinTtreeItem(rootData);
    setupModelData(data.split(QString("\n")), rootItem);
}

PoinTtreeModel::~PoinTtreeModel()
{
     delete rootItem;
}

int PoinTtreeModel::columnCount(const QModelIndex & /* parent */) const
{
    return rootItem->columnCount();
}

QVariant PoinTtreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (role != Qt::DisplayRole && role != Qt::EditRole)
        return QVariant();

    PoinTtreeItem *item = getItem(index);

    return item->data(index.column());
}

Qt::ItemFlags PoinTtreeModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return 0;

    return Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

PoinTtreeItem *PoinTtreeModel::getItem(const QModelIndex &index) const
{
    if (index.isValid()) {
        PoinTtreeItem *item = static_cast<PoinTtreeItem*>(index.internalPointer());
        if (item) return item;
    }
    return rootItem;
}

QVariant PoinTtreeModel::headerData(int section, Qt::Orientation orientation,
                               int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return rootItem->data(section);

    return QVariant();
}

QModelIndex PoinTtreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (parent.isValid() && parent.column() != 0)
        return QModelIndex();

    PoinTtreeItem *parentItem = getItem(parent);

    PoinTtreeItem *childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    else
        return QModelIndex();
}

bool PoinTtreeModel::insertColumns(int position, int columns, const QModelIndex &parent)
{
    bool success;

    beginInsertColumns(parent, position, position + columns - 1);
    success = rootItem->insertColumns(position, columns);
    endInsertColumns();

    return success;
}

bool PoinTtreeModel::insertRows(int position, int rows, const QModelIndex &parent)
{
    PoinTtreeItem *parentItem = getItem(parent);
    bool success;

    beginInsertRows(parent, position, position + rows - 1);
    success = parentItem->insertChildren(position, rows, rootItem->columnCount());
    endInsertRows();

    return success;
}

QModelIndex PoinTtreeModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    PoinTtreeItem *childItem = getItem(index);
    PoinTtreeItem *parentItem = childItem->parent();

    if (parentItem == rootItem)
        return QModelIndex();

    return createIndex(parentItem->childNumber(), 0, parentItem);
}

bool PoinTtreeModel::removeColumns(int position, int columns, const QModelIndex &parent)
{
    bool success;

    beginRemoveColumns(parent, position, position + columns - 1);
    success = rootItem->removeColumns(position, columns);
    endRemoveColumns();

    if (rootItem->columnCount() == 0)
        removeRows(0, rowCount());

    return success;
}

bool PoinTtreeModel::removeRows(int position, int rows, const QModelIndex &parent)
{
    PoinTtreeItem *parentItem = getItem(parent);
    bool success = true;

    beginRemoveRows(parent, position, position + rows - 1);
    success = parentItem->removeChildren(position, rows);
    endRemoveRows();

    return success;
}

int PoinTtreeModel::rowCount(const QModelIndex &parent) const
{
    PoinTtreeItem *parentItem = getItem(parent);

    return parentItem->childCount();
}

bool PoinTtreeModel::setData(const QModelIndex &index, const QVariant &value,
                        int role)
{
    if (role != Qt::EditRole)
        return false;

    PoinTtreeItem *item = getItem(index);
    bool result = item->setData(index.column(), value);

    if (result)
        Q_EMIT dataChanged(index, index);

    return result;
}

bool PoinTtreeModel::setHeaderData(int section, Qt::Orientation orientation,
                              const QVariant &value, int role)
{
    if (role != Qt::EditRole || orientation != Qt::Horizontal)
        return false;

    bool result = rootItem->setData(section, value);

    if (result)
        Q_EMIT headerDataChanged(orientation, section, section);

    return result;
}

void PoinTtreeModel::setupModelData(const QStringList &lines, PoinTtreeItem *parent)
{
     QList<PoinTtreeItem*> parents;
     QList<int> indentations;
     parents << parent;
     indentations << 0;

     int number = 0;

     while (number < lines.count()) {
         int position = 0;
         while (position < lines[number].length()) {
             if (lines[number].mid(position, 1) != " ")
                 break;
             position++;
         }

         QString lineData = lines[number].mid(position).trimmed();

         if (!lineData.isEmpty()) {
             // Read the column data from the rest of the line.
             QStringList columnStrings = lineData.split("\t", QString::SkipEmptyParts);
             QVector<QVariant> columnData;
             for (int column = 0; column < columnStrings.count(); ++column)
                 columnData << columnStrings[column];

             if (position > indentations.last()) {
                 // The last child of the current parent is now the new parent
                 // unless the current parent has no children.
                 if (parents.last()->childCount() > 0) {
                     parents << parents.last()->child(parents.last()->childCount()-1);
                     indentations << position;
                 }
             } else {
                 while (position < indentations.last() && parents.count() > 0) {
                     parents.pop_back();
                     indentations.pop_back();
                 }
             }

             // Append a new item to the current parent's list of children.
             PoinTtreeItem *parent = parents.last();
             parent->insertChildren(parent->childCount(), 1, rootItem->columnCount());
             for (int column = 0; column < columnData.size(); ++column)
                 parent->child(parent->childCount() - 1)->setData(column, columnData[column]);
         }

         number++;
     }
 }