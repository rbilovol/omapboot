#include <common.h>
#include <string.h>

int get_downloadsize_from_string(int count, char *string)
{

	u32 value = 0;
	u32 result = 0;
	int k = 0; int j = 0;
	int raise[8] = {0, 0, 4, 8, 12, 16, 20, 24};

	while (count > 0) {

		j =  count - 1;

		value = string[j];

		if ((value >= 48) & (value <= 57))
			value = value - '0';
		else if ((value >= 65) & (value <= 90))
			value = value - 'A' + 10;
		else
			value = value - 'a' + 10;

		if (k == 0)
			value = value * 1;
		else
			value = value * (16<<raise[k]);

		result = result + value;

		count--;
		k++;
	}

	return result;
}

