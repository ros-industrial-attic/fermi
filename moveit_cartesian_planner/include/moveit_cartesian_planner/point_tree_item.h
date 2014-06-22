 #ifndef POINTTREEITEM_H
 #define POINTTREEITEM_H

 #include <QList>
 #include <QVariant>
 #include <QVector>

 class PoinTtreeItem
 {
 public:
     PoinTtreeItem(const QVector<QVariant> &data, PoinTtreeItem *parent = 0);
     ~PoinTtreeItem();

     PoinTtreeItem *child(int number);
     int childCount() const;
     int columnCount() const;
     QVariant data(int column) const;
     bool insertChildren(int position, int count, int columns);
     bool insertColumns(int position, int columns);
     PoinTtreeItem *parent();
     bool removeChildren(int position, int count);
     bool removeColumns(int position, int columns);
     int childNumber() const;
     bool setData(int column, const QVariant &value);

 private:
     QList<PoinTtreeItem*> childItems;
     QVector<QVariant> itemData;
     PoinTtreeItem *parentItem;
 };

 #endif