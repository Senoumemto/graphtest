#pragma once
const unsigned int INT24_MAX = 8388607;

class uint24
{
protected:
    unsigned char m_Internal[3];
public:
    uint24()
    {
    }

    uint24(const unsigned int val)
    {
        *this = val;
    }

    uint24(const uint24& val)
    {
        *this = val;
    }

    operator unsigned int() const
    {
        if (m_Internal[2] & 0x80) // Is this a negative?  Then we need to siingn extend.
        {
            return (0xff << 24) | (m_Internal[2] << 16) | (m_Internal[1] << 8) | (m_Internal[0] << 0);
        }
        else
        {
            return (m_Internal[2] << 16) | (m_Internal[1] << 8) | (m_Internal[0] << 0);
        }
    }

    operator float() const
    {
        return (float)this->operator unsigned int();
    }

    uint24& operator =(const uint24& input)
    {
        m_Internal[0] = input.m_Internal[0];
        m_Internal[1] = input.m_Internal[1];
        m_Internal[2] = input.m_Internal[2];

        return *this;
    }

    uint24& operator =(const unsigned int input)
    {
        m_Internal[0] = ((unsigned char*)&input)[0];
        m_Internal[1] = ((unsigned char*)&input)[1];
        m_Internal[2] = ((unsigned char*)&input)[2];

        return *this;
    }

    /***********************************************/

    uint24 operator +(const uint24& val) const
    {
        return uint24((unsigned int)*this + (unsigned int)val);
    }

    uint24 operator -(const uint24& val) const
    {
        return uint24((unsigned int)*this - (unsigned int)val);
    }

    uint24 operator *(const uint24& val) const
    {
        return uint24((unsigned int)*this * (unsigned int)val);
    }

    uint24 operator /(const uint24& val) const
    {
        return uint24((unsigned int)*this / (unsigned int)val);
    }

    /***********************************************/

    uint24 operator +(const unsigned int val) const
    {
        return uint24((unsigned int)*this + val);
    }

    uint24 operator -(const unsigned int val) const
    {
        return uint24((unsigned int)*this - val);
    }

    uint24 operator *(const unsigned int val) const
    {
        return uint24((unsigned int)*this * val);
    }

    uint24 operator /(const unsigned int val) const
    {
        return uint24((unsigned int)*this / val);
    }

    /***********************************************/
    /***********************************************/


    uint24& operator +=(const uint24& val)
    {
        *this = *this + val;
        return *this;
    }

    uint24& operator -=(const uint24& val)
    {
        *this = *this - val;
        return *this;
    }

    uint24& operator *=(const uint24& val)
    {
        *this = *this * val;
        return *this;
    }

    uint24& operator /=(const uint24& val)
    {
        *this = *this / val;
        return *this;
    }

    /***********************************************/

    uint24& operator +=(const unsigned int val)
    {
        *this = *this + val;
        return *this;
    }

    uint24& operator -=(const unsigned int val)
    {
        *this = *this - val;
        return *this;
    }

    uint24& operator *=(const unsigned int val)
    {
        *this = *this * val;
        return *this;
    }

    uint24& operator /=(const unsigned int val)
    {
        *this = *this / val;
        return *this;
    }

    /***********************************************/
    /***********************************************/

    uint24 operator >>(const unsigned int val) const
    {
        return uint24((unsigned int)*this >> val);
    }

    uint24 operator <<(const unsigned int val) const
    {
        return uint24((unsigned int)*this << val);
    }

    /***********************************************/

    uint24& operator >>=(const unsigned int val)
    {
        *this = *this >> val;
        return *this;
    }

    uint24& operator <<=(const unsigned int val)
    {
        *this = *this << val;
        return *this;
    }

    /***********************************************/
    /***********************************************/

    operator bool() const
    {
        return (unsigned int)*this != 0;
    }

    bool operator !() const
    {
        return !((unsigned int)*this);
    }

    /***********************************************/
    /***********************************************/

    bool operator ==(const uint24& val) const
    {
        return (unsigned int)*this == (unsigned int)val;
    }

    bool operator !=(const uint24& val) const
    {
        return (unsigned int)*this != (unsigned int)val;
    }

    bool operator >=(const uint24& val) const
    {
        return (unsigned int)*this >= (unsigned int)val;
    }

    bool operator <=(const uint24& val) const
    {
        return (unsigned int)*this <= (unsigned int)val;
    }

    bool operator >(const uint24& val) const
    {
        return (unsigned int)*this > (unsigned int)val;
    }

    bool operator <(const uint24& val) const
    {
        return (unsigned int)*this < (unsigned int)val;
    }

    /***********************************************/

    bool operator ==(const unsigned int val) const
    {
        return (unsigned int)*this == val;
    }

    bool operator !=(const unsigned int val) const
    {
        return (unsigned int)*this != val;
    }

    bool operator >=(const unsigned int val) const
    {
        return (unsigned int)*this >= val;
    }

    bool operator <=(const unsigned int val) const
    {
        return (unsigned int)*this <= val;
    }

    bool operator >(const unsigned int val) const
    {
        return ((unsigned int)*this) > val;
    }

    bool operator <(const unsigned int val) const
    {
        return (unsigned int)*this < val;
    }

    /***********************************************/
    /***********************************************/
};