#include "Decoder.h"

#include "datastructure/Tag.h"

namespace pipeline {

Decoder::Decoder()
{

}

void Decoder::loadSettings(decoder_settings_t &&settings)
{

}

std::vector<Tag> Decoder::process(std::vector<Tag> &&taglist)
{


	return std::move(taglist);
}

}
