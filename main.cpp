/* Copyright (c) EcarX - All Rights Reserved
 * Author: Ke Ma <ke.ma@ecarx.com.cn>
 * description: cloudpoints projection main function
 */

#include "combData.hpp"
#include <iostream>

int main(int argc, char **argv)
{

	CombData *cd = new CombData();

	if (!cd->exec())
	{
		std::cerr << "\t mk11-error: something wrong" << std::endl;
	}

	delete cd;

	return 0;
}
