#ifndef TEXTUREMANAGER_H
#define TEXTUREMANAGER_H

#include <QObject>
#include <QMultiMap>

namespace Marble
{
    class MarbleMap;
}

class TextureSetting
{
public:
    TextureSetting(const QString& source, int level);
    bool operator==(const TextureSetting &other) const;

    QString source() const;

    void setKey(const QString &key);
    QString key() const;

    int level() const;
    bool active() const;
    bool visible() const;

    void setActive(bool active);
    void setVisible(bool visible);
private:
    QString mSource;
    QString mKey;
    int mLevel;
    bool mActive;
    bool mVisible;
};

class TextureManager : public QObject
{
Q_OBJECT
public:
    TextureManager(Marble::MarbleMap *marbleMap);
    void addTextureLayer(const TextureSetting& textureLayer);
    void updateTextures();

public slots:
    void toggle();
    void toggle(int level);

private:
    void nextLevel();

    typedef QMultiMap<int, TextureSetting> TextureLevelMap;
    TextureLevelMap mTextures;
    bool mLayersVisible;
    Marble::MarbleMap *mMarbleMap;
    int mLevel;
};

#endif // TEXTUREMANAGER_H
