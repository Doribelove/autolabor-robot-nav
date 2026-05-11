//
// Created by ydrml on 2019/2/28.
//

#ifndef AUTOLABOR_CANBUS_DRIVER_TRANSFORM_H
#define AUTOLABOR_CANBUS_DRIVER_TRANSFORM_H

#include <vector>
#include <algorithm>

namespace {
	/**
      * 转换器
      *
      * @tparam t 内容类型
      */
	template<class t>
	union msg_union {
		uint8_t bytes[sizeof(t)];
		t       data;
	};
}

namespace autolabor {
	/**
	 * 按大端字节序将字节数组转换到数据类型
	 *
	 * @tparam t     数据类型
	 * @param buffer 字节数组
	 * @return       数据
	 */
	template<class t>
	inline t build(const uint8_t *buffer) {
		msg_union<t> transformer;
		std::reverse_copy(buffer, buffer + sizeof(t), transformer.bytes);
		return transformer.data;
	}

    /**
	 * 按小端字节序将字节数组转换到数据类型
	 *
	 * @tparam t     数据类型
	 * @param buffer 字节数组
	 * @return       数据
	 */
    template<class T>
    inline T build_from_little_endian(const uint8_t *buffer) {
        msg_union<T> transformer;
        std::memcpy(transformer.bytes, buffer, sizeof(T));
        return transformer.data;
    }
	
	/**
	  * 按大端字节序将字节数组转换到数据类型
	  *
	  * @tparam t     数据类型
	  * @param data 数据
	 * @param buffer 字节数组
	  */
	template<class t>
	inline void pack_into(t data, uint8_t *buffer) {
		msg_union<t> temp{};
		temp.data = data;
		
		std::reverse_copy(temp.bytes, temp.bytes + sizeof(t), buffer);
	}

    /**
	  * 按小端字节序将字节数组转换到数据类型
	  *
	  * @tparam t     数据类型
	  * @param data 数据
	 * @param buffer 字节数组
	  */
    template<class T>
    inline void pack_into_little_endian(T data, uint8_t *buffer) {
        msg_union<T> temp{};
        temp.data = data;

        std::memcpy(buffer, temp.bytes, sizeof(T));
    }
	
	/**
	  * 按大端字节序将字节数组转换到数据类型
	  *
	  * @tparam t   数据类型
	  * @param data 数据
	  * @return     字节数组
	  */
	template<class t>
	inline std::vector <uint8_t> pack(t data) {
		std::vector <uint8_t> buffer(sizeof(t));
		pack_into(data, buffer.data());
		return buffer;
	}

    /**
	  * 按小端字节序将字节数组转换到数据类型
	  *
	  * @tparam t   数据类型
	  * @param data 数据
	  * @return     字节数组
	  */
    template<class t>
    inline std::vector<uint8_t> pack_little_endian(t data) {
        std::vector<uint8_t> buffer(sizeof(t));
        pack_into_little_endian(data, buffer.data());
        return buffer;
    }
}

#endif //AUTOLABOR_CANBUS_DRIVER_TRANSFORM_H
