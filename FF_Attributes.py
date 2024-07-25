import Rhino as rh
import System

def getObjectProperties():
    prop = rh.DocObjects.ObjectAttributes()
    prop.ObjectColor = System.Drawing.Color.FromArgb(0, 0, 255)
    prop.ColorSource = rh.DocObjects.ObjectColorSource.ColorFromObject
    return prop


if __name__ == '__main__':
    getObjectProperties()