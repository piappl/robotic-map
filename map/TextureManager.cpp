#include <marble/MarbleMap.h>
#include <marble/MarbleDirs.h>
#include <marble/GeoSceneTextureTileDataset.h>
#include "TextureManager.h"

using namespace Marble;

namespace
{
    const QString textureOverlayKeyPrefix = "overlay_";
    const QString aerialTexture = "aerial";
    const QString planetPrefix = "earth";

    QString getMapsDirectory()
    {
        return MarbleDirs::path("maps/" + planetPrefix);
    }

    QStringList detectOverlays()
    {
        QDir mapsDirectory(getMapsDirectory());

        QStringList filters;
        filters << aerialTexture << textureOverlayKeyPrefix + "*";

        //Return all subdirectories in mapsDirectory that match filters, ordered by name
        return mapsDirectory.entryList(filters, QDir::Dirs, QDir::Name);
    }

    GeoSceneTextureTileDataset* makeTexture(TextureSetting textureSetting)
    {   //This might read some kind of file or setting in the future
        GeoSceneTextureTileDataset *texture = new GeoSceneTextureTileDataset(textureSetting.source());
        //texture->setProjection(GeoSceneTiled::Mercator);
        texture->setFileFormat("PNG");
        texture->setBlending("AlphaBlending");
        texture->setTileSize(QSize(256,256));
        texture->setMaximumTileLevel(19);
        texture->setLevelZeroColumns(1);
        texture->setLevelZeroRows(1);
        texture->setStorageLayout(GeoSceneTileDataset::TileMapService);
        texture->setSourceDir(textureSetting.source());

        /*
        if (texture->sourceDir().endsWith(aerialTexture, Qt::CaseInsensitive))
        {   //TODO - read it from a config file instead
            GeoDataLatLonBox box(52.3626, 52.069, 21.283, 20.83237, GeoDataCoordinates::Degree);
            texture->setLatLonBox(box);
        }
        */

        return texture;
    }
}

TextureSetting::TextureSetting(const QString &source, int level)
    : mSource(source), mLevel(level), mActive(false), mVisible(false)
{
}

bool TextureSetting::operator==(const TextureSetting& other) const
{
    return source() == other.source()
           && level() == other.level()
           && key() == other.key();
}

QString TextureSetting::source() const { return mSource; }
void TextureSetting::setKey(const QString &key) { mKey = key; }
QString TextureSetting::key() const { return mKey; }
int TextureSetting::level() const { return mLevel; }
bool TextureSetting::active() const { return mActive; }
bool TextureSetting::visible() const { return mVisible; }
void TextureSetting::setActive(bool active) { mActive = active; }
void TextureSetting::setVisible(bool visible) { mVisible = visible; }

TextureManager::TextureManager(MarbleMap *marbleMap)
    : mLayersVisible(false),
      mMarbleMap(marbleMap),
      mLevel(0)
{
    QStringList overlays = detectOverlays();
    int layerNumber = 1;
    foreach(QString overlay, overlays)
    {
        QString relativeSourceDir = planetPrefix + "/" + overlay;
        addTextureLayer(TextureSetting(relativeSourceDir, layerNumber));
        layerNumber++;
    }

    updateTextures();
}

void TextureManager::addTextureLayer(const TextureSetting &textureLayer)
{
    int level = textureLayer.level();
    if (!mTextures.contains(level, textureLayer))
    {
        mTextures.insertMulti(level, textureLayer);
    }
}

void TextureManager::updateTextures()
{
    for (TextureLevelMap::iterator it = mTextures.begin(); it != mTextures.end(); ++it)
    {
        TextureSetting &texture = it.value();
        if (texture.visible() && !texture.active())
        {
            texture.setActive(true);
            GeoSceneTextureTileDataset *textureToUpdate = makeTexture(texture);
            QString key = mMarbleMap->addTextureLayer(textureToUpdate); //Object lifetime is managed by Marble
            texture.setKey(key);
        }
        else if (!texture.visible() && texture.active())
        {
            texture.setActive(false);
            mMarbleMap->removeTextureLayer(texture.key());
        }
    }
}

void TextureManager::toggle()
{
    nextLevel();
    for (TextureLevelMap::iterator it = mTextures.begin(); it != mTextures.end(); ++it)
    {
        TextureSetting &texture = it.value();
        texture.setVisible(texture.level() <= mLevel);
    }
    updateTextures();
}

void TextureManager::nextLevel()
{
    QList<int> availableLevels = mTextures.keys();
    int former = 0;
    bool found = false;
    foreach (int level, availableLevels)
    {
        if (mLevel == former)
        {
            mLevel = level;
            found = true;
            break;
        }
        former = level;
    }
    if (!found)
    {
        mLevel = 0;
    }
}

void TextureManager::toggle(int level)
{
    for (TextureLevelMap::iterator it = mTextures.begin(); it != mTextures.end(); ++it)
    {
        if (it.key() == level)
        {
            TextureSetting &texture = it.value();
            texture.setVisible(!texture.visible());
        }
    }
    updateTextures();
}
