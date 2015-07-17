package com.firos.kiara.connector;

import java.util.Objects;
import java.io.IOException;

import org.fiware.kiara.serialization.impl.Serializable;
import org.fiware.kiara.serialization.impl.SerializerImpl;
import org.fiware.kiara.serialization.impl.BinaryInputStream;
import org.fiware.kiara.serialization.impl.BinaryOutputStream;

public class Message implements Serializable
{
    @Override
    public void serialize(SerializerImpl impl, BinaryOutputStream message,
            String name) throws IOException
    {
        impl.serializeString(message, name, this.str_);
    }

    @Override
    public void deserialize(SerializerImpl impl, BinaryInputStream message,
            String name) throws IOException
    {
        this.str_ = impl.deserializeString(message, name);
    }

    @Override
    public boolean equals(Object other)
    {
        if(other instanceof Message)
        {
            if(this.str_.compareTo(((Message)other).str_) == 0)
                return true;
        }

        return false;
    }

    @Override
    public int hashCode()
    {
        return Objects.hash(this.str_);
    }

    public String getStr()
    {
        return str_;
    }

    public void setStr(String str)
    {
        str_ = str;
    }

    public void copy(Message other)
    {
        this.str_ = other.str_;
    }

    private String str_ = "";
}