#include "PNTWriter.h"

PNTWriter::PNTWriter(string file, AABB, double scale, BatchTable bt, FeatureTable ft)
{
	writer = new std::ofstream(file, ios::out | ios::binary);
}

PNTWriter::~PNTWriter()
{
	close();
}

/*
Writes a Point to the Batch Table
*/
void PNTWriter::write(const Point & point)
{
	points_length += 1;


	//add point to BatchTable
}

std::vector<char> PNTWriter::createHeader() const
{
	std::vector<char> buffer;

	for (auto &const c : magic) { // 4bytes
		buffer.push_back(c);
	}

	buffer.push_back(version); // 1 byte .. 
	buffer.push_back(t_byteLength);
	buffer.push_back(ftJSON_byteLength);
	buffer.push_back(ft_byteLength);
	buffer.push_back(btJSON_byteLength);
	buffer.push_back(bt_byteLength);

	return buffer;
}


void PNTWriter::writePNT()
{
	//body
	//set byteLength
	//create header
	//write to disk
}

void PNTWriter::close()
{
	if (writer != NULL) {
		writer->close();
		delete writer;
		writer = NULL;
	}
}

uint32_t PNTWriter::getbyteLength() const
{
	uint32_t len;

	return len;
}

 
