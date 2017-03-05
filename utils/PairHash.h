//
//  This file is licensed under the MIT license.
//

#ifndef PairHash_H
#define PairHash_H

namespace std {
	template <> struct hash<std::pair<double, double>>
	{
		size_t operator()(const std::pair<double, double> & x) const
		{
			return std::hash<double>()(x.first)^(std::hash<double>()(x.second)<<16);
		}
	};
}

#endif //PairHash_H