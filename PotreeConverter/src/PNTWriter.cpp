
#include "PNTWriter.h"
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

template<typename T>
void writeAsBytes(const T& obj, std::vector<std::byte>& vec) {
	const auto mem = reinterpret_cast<const std::byte*>(&obj);
	const auto size = sizeof(T);
	vec.reserve(vec.size() + size);
	for (size_t idx = 0; idx < size; ++idx) {
		vec.push_back(mem[idx]);
	}
}

PNTWriter::PNTWriter(string file, AABB aabb, double scale) 
	: scale(scale), aabb(aabb), file(file)
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
	featuretable.POINTS_LENGTH += 1;
	// add point to positionBinary
	positions.push_back(point.position.x);
	positions.push_back(point.position.y);
	positions.push_back(point.position.z);

	colors.push_back(point.color.x);
	colors.push_back(point.color.y);
	colors.push_back(point.color.z);
	// with Potree::Point many features are not available

}

void PNTWriter::writeHeader()
{
	// 28 bytes
	std::vector<std::byte> buffer;

	for (const auto& c : magic) { // 4 Bytes
		writeAsBytes(c, buffer);
	}
	writeAsBytes(version, buffer); // 4 Bytes...
	writeAsBytes(t_byteLength, buffer); 
	writeAsBytes(ftJSON_byteLength, buffer);
	writeAsBytes(ft_byteLength, buffer);
	writeAsBytes(btJSON_byteLength, buffer);
	writeAsBytes(bt_byteLength, buffer);
	

	for (int i = 0; i < buffer.size(); i++) {
		writer->write(reinterpret_cast<const char*>(&buffer[i]), sizeof(char));
	}

}

int PNTWriter::get_number_of_digits(int integer)
{
	return integer > 0 ? (int)log10((double)integer) + 1 : 1;
}

void PNTWriter::writePNT()
{
	// 28 bytes header size
	writer->seekp(28);
	// create batch and featuretable (and get the size)
	std::vector<std::byte> ft_binbody = createFeatureBIN(); // the features in this buffer have to be alligned
	// ft_bytelength
	// create all BIN files and get the bytesize (position_bteLength)

	/*
	ftJSON_byteLength has to be a multiple of 4 
	*/

	// here we must have all featuretable assets set up
	
	
	
	// all correct byteoffsets are set. the first byteoffset is 0
	
	// now ftJSON_byteLength should be a multiple of 4
	// we do this to set an (%4)offset for float => Start offset of Float32Array should be a multiple of 4
	// TODO: This only works for POSITION 
	
	featuretable.POSITION_byteOffset.emplace(0);
	const int jsonBL = seekfeatureTableJSONByteSize();
	int bytes_to_fill = 0;
	if (jsonBL % 4 != 0) {
		bytes_to_fill = 4 - (jsonBL % 4);
		
		writer->seekp(28 + jsonBL);
		std::byte dummy = (std::byte)32; // space
		for (int i = 0; i < bytes_to_fill; i++) {
			writer->write(reinterpret_cast<const char*>(&dummy), sizeof(std::byte));
		}
		featuretable.POSITION_byteOffset.emplace(bytes_to_fill);
		ftJSON_byteLength = jsonBL; // header
	}
	else
	{
		ftJSON_byteLength = jsonBL; // header
	}
	
	writer->seekp(28);
	writeFeatureTableJSON();
	 //writer at "}" -> write zero(s) and  "}"
	/*
	if (bytes_to_fill != 0) {
		writer->seekp(28 + jsonBL - 1);
		std::byte dummy = (std::byte)0;
		for (int i = 0; i < bytes_to_fill; i++) {
			writer->write(reinterpret_cast<const char*>(&dummy), sizeof(std::byte));
		}
		std::byte end = (std::byte)'}';
		writer->write(reinterpret_cast<const char*>(&end), sizeof(std::byte));
	}
	*/

	writer->seekp(28 + jsonBL + bytes_to_fill);
	//ft_bin
	for (int i = 0; i < ft_binbody.size(); i++) {
		writer->write(reinterpret_cast<const char*>(&ft_binbody[i]), sizeof(std::byte));
	}
	//create header
	writer->seekp(0);
	t_byteLength = 28 + ft_byteLength + ftJSON_byteLength + bt_byteLength + btJSON_byteLength;
	writeHeader();
}

void PNTWriter::close()
{
	if (writer != NULL) {
		writer->close();
		delete writer;
		writer = NULL;
	}
}


std::vector<std::byte> PNTWriter::createFeatureBIN()//create a featuretable before this method
{
	std::vector<std::byte> buffer;
	ft_byteLength = 0;
	// write points to binary
	// positions.size() and colors.size() should be equal

	//The most efficient way
	const auto posData = positions.data();
	buffer.resize(positions.size() * sizeof(float));
	std::memcpy(buffer.data(), posData, positions.size() * sizeof(float));

	int bsf = sizeof(float);
	int bsi8 = sizeof(uint8_t);
	//featuretable.POSITION_byteOffset = ft_byteLength;
	for (int i = 0; i < positions.size(); i++) {
		
		//writer->write(reinterpret_cast<const char*>(&positions[i]), bsf);
		position_byteLength += bsf; // important for the offset
		ft_byteLength += bsf;
		number_of_features += 1;
	}
	/*
	featuretable.RGB_byteOffset = ft_byteLength;
	for (int i = 0; i < colors.size(); i++) {
		
		//writer->write(reinterpret_cast<const char*>(&colors[i]), bsi8);
		writeAsBytes(colors[i], buffer);
		// set the byteoffset 

		rgb__byteLength += bsi8;
		ft_byteLength += bsi8;
	}
	*/
	// TODO: add more features 

	return buffer;
}


// not that pretty but solves problem with rapidjson allocator
int PNTWriter::seekfeatureTableJSONByteSize()
{
	// https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/pnts.featureTable.schema.json
	Document document;
	document.SetObject();
	Document::AllocatorType& alloc = document.GetAllocator();

	// Point semantics Value must be a REFERENCE to the binary body (byte offset)
	if (featuretable.POSITION_byteOffset != nullopt) {
		Value obj(kObjectType);
		int bo = featuretable.POSITION_byteOffset.value();
		obj.AddMember("byteOffset", bo, alloc);
		document.AddMember("POSITION", obj, alloc);
	}
	if (featuretable.POSITION_QUANTIZED_byteOffset != nullopt) { // if this is set QUANTIZED_VOLUME_OFFSET","QUANTIZED_VOLUME_SCALE" have to be set

	}
	if (featuretable.RGB_byteOffset != nullopt) {

	}
	if (featuretable.RGBA_byteOffset != nullopt) {

	}
	if (featuretable.RGB565_byteOffset != nullopt) {

	}
	if (featuretable.NORMAL_byteOffset != nullopt) {

	}
	if (featuretable.NORMAL_OCT16P_byteOffset != nullopt) {

	}
	if (featuretable.BATCH_ID_byteOffset != nullopt) {

	}

	// Global semantics Value is an actual value here
	document.AddMember("POINTS_LENGTH", featuretable.POINTS_LENGTH, alloc);

	// using deep copy move semantics
	if (featuretable.RTC_CENTER != nullopt) {
		Vector3 rtc_val = featuretable.RTC_CENTER.value();
		Value rtc_arr(rapidjson::kArrayType);
		rtc_arr.PushBack(rtc_val.x, alloc);
		rtc_arr.PushBack(rtc_val.y, alloc);
		rtc_arr.PushBack(rtc_val.z, alloc);
		document.AddMember("RTC_CENTER", rtc_arr, alloc);
	}
	if (featuretable.QUANTIZED_VOLUME_OFFSET != nullopt) {
		Vector3 qvo_val = featuretable.QUANTIZED_VOLUME_OFFSET.value();
		Value qvo_arr(rapidjson::kArrayType);
		qvo_arr.PushBack(qvo_val.x, alloc);
		qvo_arr.PushBack(qvo_val.y, alloc);
		qvo_arr.PushBack(qvo_val.z, alloc);
		document.AddMember("QUANTIZED_VOLUME_OFFSET", qvo_arr, alloc);
	}
	if (featuretable.QUANTIZED_VOLUME_SCALE != nullopt) {
		Vector3 qvs_val = featuretable.QUANTIZED_VOLUME_SCALE.value();
		Value qvs_arr(rapidjson::kArrayType);
		qvs_arr.PushBack(qvs_val.x, alloc);
		qvs_arr.PushBack(qvs_val.y, alloc);
		qvs_arr.PushBack(qvs_val.z, alloc);
		document.AddMember("QUANTIZED_VOLUME_SCALE", qvs_arr, alloc);
	}
	if (featuretable.CONSTANT_RGBA != nullopt) {
		RGBA c_rgba = featuretable.CONSTANT_RGBA.value();
		Value rgba_arr(rapidjson::kArrayType);
		rgba_arr.PushBack(c_rgba.R, alloc);
		rgba_arr.PushBack(c_rgba.G, alloc);
		rgba_arr.PushBack(c_rgba.B, alloc);
		rgba_arr.PushBack(c_rgba.A, alloc);
		document.AddMember("CONSTANT_RGBA", rgba_arr, alloc);
	}
	if (featuretable.BATCH_LENGTH != nullopt) {
		document.AddMember("BATCH_LENGTH", featuretable.BATCH_LENGTH.value(), alloc);
	}
	rapidjson::StringBuffer jsonBuffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(jsonBuffer);

	document.Accept(writer);

	auto jsonSize = jsonBuffer.GetSize();
	return jsonSize;
}

void PNTWriter::writeFeatureTableJSON()
{
	// https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/pnts.featureTable.schema.json
	Document document;
	document.SetObject();
	Document::AllocatorType& alloc = document.GetAllocator();

	// Point semantics Value must be a REFERENCE to the binary body (byte offset)
	if (featuretable.POSITION_byteOffset != nullopt) {
		Value obj(kObjectType);
		int bo = featuretable.POSITION_byteOffset.value();
		obj.AddMember("byteOffset", bo, alloc);
		document.AddMember("POSITION", obj, alloc);
	}
	if (featuretable.POSITION_QUANTIZED_byteOffset != nullopt) { // if this is set QUANTIZED_VOLUME_OFFSET","QUANTIZED_VOLUME_SCALE" have to be set

	}
	if (featuretable.RGB_byteOffset != nullopt) {

	}
	if (featuretable.RGBA_byteOffset != nullopt) {

	}
	if (featuretable.RGB565_byteOffset != nullopt) {

	}
	if (featuretable.NORMAL_byteOffset != nullopt) {

	}
	if (featuretable.NORMAL_OCT16P_byteOffset != nullopt) {

	}
	if (featuretable.BATCH_ID_byteOffset != nullopt) {

	}

	// Global semantics Value is an actual value here
	document.AddMember("POINTS_LENGTH", featuretable.POINTS_LENGTH, alloc);

	// using deep copy move semantics
	if (featuretable.RTC_CENTER != nullopt) {
		Vector3 rtc_val = featuretable.RTC_CENTER.value();
		Value rtc_arr(rapidjson::kArrayType);
		rtc_arr.PushBack(rtc_val.x, alloc);
		rtc_arr.PushBack(rtc_val.y, alloc);
		rtc_arr.PushBack(rtc_val.z, alloc);
		document.AddMember("RTC_CENTER", rtc_arr, alloc);
	}
	if (featuretable.QUANTIZED_VOLUME_OFFSET != nullopt) {
		Vector3 qvo_val = featuretable.QUANTIZED_VOLUME_OFFSET.value();
		Value qvo_arr(rapidjson::kArrayType);
		qvo_arr.PushBack(qvo_val.x, alloc);
		qvo_arr.PushBack(qvo_val.y, alloc);
		qvo_arr.PushBack(qvo_val.z, alloc);
		document.AddMember("QUANTIZED_VOLUME_OFFSET", qvo_arr, alloc);
	}
	if (featuretable.QUANTIZED_VOLUME_SCALE != nullopt) {
		Vector3 qvs_val = featuretable.QUANTIZED_VOLUME_SCALE.value();
		Value qvs_arr(rapidjson::kArrayType);
		qvs_arr.PushBack(qvs_val.x, alloc);
		qvs_arr.PushBack(qvs_val.y, alloc);
		qvs_arr.PushBack(qvs_val.z, alloc);
		document.AddMember("QUANTIZED_VOLUME_SCALE", qvs_arr, alloc);
	}
	if (featuretable.CONSTANT_RGBA != nullopt) {
		RGBA c_rgba = featuretable.CONSTANT_RGBA.value();
		Value rgba_arr(rapidjson::kArrayType);
		rgba_arr.PushBack(c_rgba.R, alloc);
		rgba_arr.PushBack(c_rgba.G, alloc);
		rgba_arr.PushBack(c_rgba.B, alloc);
		rgba_arr.PushBack(c_rgba.A, alloc);
		document.AddMember("CONSTANT_RGBA", rgba_arr, alloc);
	}
	if (featuretable.BATCH_LENGTH != nullopt) {
		document.AddMember("BATCH_LENGTH", featuretable.BATCH_LENGTH.value(), alloc);
	}

	rapidjson::StringBuffer jsonBuffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(jsonBuffer);

	document.Accept(writer);

	auto jsonInMemory = jsonBuffer.GetString();
	auto jsonSize = jsonBuffer.GetSize();

	//Write JSON memory into ofstream
	this->writer->write(jsonInMemory, jsonSize);

}
