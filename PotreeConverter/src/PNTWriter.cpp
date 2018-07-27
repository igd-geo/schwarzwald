
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
	/*
	colors.push_back(point.color.x);
	colors.push_back(point.color.y);
	colors.push_back(point.color.z);
	*/
	colors.push_back(255);
	colors.push_back(255);
	colors.push_back(255);
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
	
	int jsonBL = seekfeatureTableJSONByteSize();
	// byteoffset of the first featureArray is set to 0 by deafault
	
	if (jsonBL % 4 != 0) {

		// start padding
		int padding = 4 - (jsonBL % 4);

		writer->seekp(28 + jsonBL);
		std::byte dummy = (std::byte)32; // space (for json-parsers)
		for (int i = 0; i < padding; i++) {
			writer->write(reinterpret_cast<const char*>(&dummy), sizeof(std::byte));
		}
		ftJSON_byteLength = jsonBL + padding; // header
	}
	else // no padding
	{
		ftJSON_byteLength = jsonBL;
	}
	
	writer->seekp(28);
	writeFeatureTableJSON();
	
	writer->seekp(28 + ftJSON_byteLength);


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
	int bsf = sizeof(float);
	int bsi8 = sizeof(uint8_t);

	ft_byteLength = 0;
	number_of_features = 0;

	assert(positions.size() == colors.size());
	//The most efficient way
	
	if (positions.size() != 0) { // if positions.size() != 0?
		const auto posData = positions.data();
		buffer.resize(positions.size() * sizeof(float));
		std::memcpy(buffer.data(), posData, positions.size() * sizeof(float));

		position_byteLength = positions.size() * sizeof(float);
		ft_byteLength += position_byteLength;

		featuretable.POSITION_byteOffset.emplace(0);
		number_of_features += 1;
	}
	if (false) { // if this is set QUANTIZED_VOLUME_OFFSET","QUANTIZED_VOLUME_SCALE" have to be set

		
	}
	if (colors.size() != 0) {
		const auto rgbData = colors.data();
		auto be = colors.size();
		auto end = buffer.size();
		buffer.resize(buffer.size() + colors.size() * sizeof(uint8_t));
		auto la = buffer.size();
		std::memcpy(&buffer.at(end), rgbData, colors.size() * sizeof(uint8_t));

		rgb_byteLength = colors.size() * sizeof(uint8_t);
		ft_byteLength += rgb_byteLength;

		featuretable.RGB_byteOffset.emplace(position_byteLength + position_quantized_byteLength);
		number_of_features += 1;
	}
	/*
	if (featuretable.RGBA_byteOffset != nullopt) {

	}
	if (featuretable.RGB565_byteOffset != nullopt) {

	}
	if (featuretable.NORMAL_byteOffset != nullopt) {

	}
	if (featuretable.NORMAL_OCT16P_byteOffset != nullopt) {

	}
	if (featuretable.BATCH_ID_byteOffset != nullopt) {

	}*/
	
	return buffer;
}


// not that pretty but skips a problem with rapidjson allocator
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
		Value obj(kObjectType);
		int bo = featuretable.RGB_byteOffset.value();
		obj.AddMember("byteOffset", bo, alloc);
		document.AddMember("RGB", obj, alloc);
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
		Value obj(kObjectType);
		int bo = featuretable.RGB_byteOffset.value();
		obj.AddMember("byteOffset", bo, alloc);
		document.AddMember("RGB", obj, alloc);
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
