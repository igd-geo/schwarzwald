#pragma once

struct PointBuffer;

/**
 * Interface for writers that take points and convert them into Potree octree structure
 */
struct IWriter
{
  virtual ~IWriter() {}

  virtual void cache(const PointBuffer& points) = 0;
  virtual void index() = 0;
  virtual bool needs_indexing() const = 0;
  virtual void wait_until_indexed() = 0;
  virtual void flush() = 0;
  virtual bool needs_flush() const = 0;
  virtual void close() = 0;
};