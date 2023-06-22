#pragma once

#include"general.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include "bmp.h"


using dmod = mod<double>;
using dtri = tri<double>;
using dvec3 = vec3<double>;
using hmat3 = Eigen::Matrix3<halff>;
//�e�X�g��1x1�̐����`������-�P�ɍ��
int MakeTestSquare(dmod& ret);

void ModLoader(const std::string& path, dmod& ret);

template<size_t res>void PrintBmp(const std::string& path,const bitmap<res>& dat) {
	sptr<img> g(new img);
	g->width = res;
	g->height = res;

	for (int y = 0; y < g->height; y++)
		for (int x = 0; x < g->width; x++) {
			g->data[g->height-1-y][x].r = dat.at(y * res + x).at(0)*255;
			g->data[g->height-1-y][x].g = dat.at(y * res + x).at(1)*255;
			g->data[g->height-1-y][x].b = dat.at(y * res + x).at(2)*255;
		}
	WriteBmp(path.c_str(), g.get());
}

//�T�C�Y���w�肵�ĕ`��@�N���b�v�Ȃǂ͂Ȃ����������珑���o�����������炠�܂�g��Ȃ���
template<size_t res>void PrintBmpWithAnotherSize_YOU_MUST_READ_COMMENT(const std::string& path, const bitmap<res>& dat,size_t width,size_t height) {
	sptr<img> g(new img);
	g->width = width;
	g->height = height;

	auto datite = dat.cbegin();
	for (int y = 0; y < g->height; y++)
		for (int x = 0; x < g->width; x++) {

			g->data[g->height - 1 - y][x].r = (*datite).at(0) * 255;
			g->data[g->height - 1 - y][x].g = (*datite).at(1) * 255;
			g->data[g->height - 1 - y][x].b = (*datite).at(2) * 255;

			datite++;
		}
	WriteBmp(path.c_str(), g.get());
}

struct attribRez {
	Eigen::Vector3<halff> norm;
	Eigen::Vector3<halff> hitpoint;
	ray refrect;
};
attribRez Attrib(const closesthit& att, const las* ptlas);

hmat4 MakeTranslate(const hvec3& v);
hmat4 MakeScale(const hvec3& v);
hmat4 MakeScale(const halff& s);
hmat4 MakeEular(const hvec3& r);

template<typename ATTRIB_TYPE>void MakeDammyAttributes(size_t siz, toolkit::triangleAttributesFramework<ATTRIB_TYPE>& triAttribs) {
	using namespace std;

	cout << "attrib size " << siz << endl;

	//��������
	for (size_t i = 0; i < siz; i++) {
		toolkit::triangleAttributeFramework<ATTRIB_TYPE>& triAttrib = triAttribs.at(i);

		for (size_t j = 0; j < 3; j++) {
			ATTRIB_TYPE& attrib = triAttrib.at(j);
			
			attrib.at(0) = 0;
			attrib.at(1) = 1;
			attrib.at(2) = 0;
		}
	}

}