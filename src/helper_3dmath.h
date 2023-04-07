// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <cmath>
#include "inttypes.h"

class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;
        
        Quaternion();
        
        Quaternion(float nw, float nx, float ny, float nz);

        Quaternion getProduct(Quaternion q);
        Quaternion getConjugate();
        float getMagnitude();
        void normalize();
        
        Quaternion getNormalized();
};

class VectorInt16 {
    public:
        int16_t x;
        int16_t y;
        int16_t z;

        VectorInt16();
        
        VectorInt16(int16_t nx, int16_t ny, int16_t nz);

        float getMagnitude();
        void normalize();
        VectorInt16 getNormalized();
        
        void rotate(Quaternion *q);
        VectorInt16 getRotated(Quaternion *q);
};

class VectorFloat {
    public:
        float x;
        float y;
        float z;

        VectorFloat();
        
        VectorFloat(float nx, float ny, float nz);
        
        float getMagnitude();
        void normalize();
        
        VectorFloat getNormalized();
        
        void rotate(Quaternion *q);
        
        VectorFloat getRotated(Quaternion *q);
        
};

#endif /* _HELPER_3DMATH_H_ */