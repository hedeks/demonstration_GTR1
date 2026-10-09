#include "depth_filter.hpp"
#include <cmath>

namespace gtr1 {
namespace core {

DepthFilter::DepthFilter(int diameter,
                         double sigma_depth_m,
                         double sigma_space_px,
                         float min_range,
                         float max_range,
                         float depth_scale)
    : diameter_(diameter),
      sigma_depth_(sigma_depth_m),
      sigma_space_(sigma_space_px),
      min_range_m_(min_range),
      max_range_m_(max_range),
      depth_scale_(depth_scale),
      remove_flying_pixels_(true),
      remove_flying_edges_(true),
      fill_small_holes_(true),
      edge_gradient_threshold_(0.08f) {}

void DepthFilter::set_parameters(int diameter, double sigma_depth_m, double sigma_space_px) {
    diameter_ = diameter;
    sigma_depth_ = sigma_depth_m;
    sigma_space_ = sigma_space_px;
}

cv::Mat DepthFilter::filter(const cv::Mat& raw_depth_mm) const {
    if (raw_depth_mm.empty()) return cv::Mat();

    CV_Assert(raw_depth_mm.type() == CV_16UC1);

    // 1. Преобразование в float (метры) с учетом шкалы сенсора (TUM standard: 5000.0)
    cv::Mat depth_m;
    raw_depth_mm.convertTo(depth_m, CV_32FC1, 1.0f / depth_scale_);

    // 2. Маска валидных значений по дистанции
    cv::Mat valid_mask = (depth_m >= min_range_m_) & (depth_m <= max_range_m_);

    cv::Mat masked_depth = cv::Mat::zeros(depth_m.size(), CV_32FC1);
    depth_m.copyTo(masked_depth, valid_mask);

    // 2.1. Заполнение микро-проколов (1-3 px) в связных областях
    if (fill_small_holes_) {
        int rows = masked_depth.rows;
        int cols = masked_depth.cols;
        for (int r = 1; r < rows - 1; ++r) {
            float* curr = masked_depth.ptr<float>(r);
            const float* prev = masked_depth.ptr<float>(r - 1);
            const float* next = masked_depth.ptr<float>(r + 1);
            for (int c = 1; c < cols - 1; ++c) {
                if (curr[c] <= 0.0f) {
                    float vals[4];
                    int count = 0;
                    if (prev[c] >= min_range_m_ && prev[c] <= max_range_m_) vals[count++] = prev[c];
                    if (next[c] >= min_range_m_ && next[c] <= max_range_m_) vals[count++] = next[c];
                    if (curr[c-1] >= min_range_m_ && curr[c-1] <= max_range_m_) vals[count++] = curr[c-1];
                    if (curr[c+1] >= min_range_m_ && curr[c+1] <= max_range_m_) vals[count++] = curr[c+1];
                    if (count >= 3) {
                        float sum = 0.0f;
                        for (int i = 0; i < count; ++i) sum += vals[i];
                        curr[c] = sum / static_cast<float>(count);
                    }
                }
            }
        }
        valid_mask = (masked_depth >= min_range_m_) & (masked_depth <= max_range_m_);
    }

    // 3. Двусторонний фильтр (Bilateral Filter)
    cv::Mat filtered_m;
    cv::bilateralFilter(masked_depth, filtered_m, diameter_, sigma_depth_, sigma_space_);

    // 3.1. Детекция и подавление летающих граней (Flying Edges) на разрывах контуров
    if (remove_flying_edges_) {
        int rows = filtered_m.rows;
        int cols = filtered_m.cols;
        cv::Mat edge_mask = cv::Mat::zeros(rows, cols, CV_8UC1);

        for (int r = 1; r < rows - 1; ++r) {
            const float* prev = filtered_m.ptr<float>(r - 1);
            const float* curr = filtered_m.ptr<float>(r);
            const float* next = filtered_m.ptr<float>(r + 1);
            uint8_t* mask_row = edge_mask.ptr<uint8_t>(r);

            for (int c = 1; c < cols - 1; ++c) {
                float d = curr[c];
                if (d <= min_range_m_) continue;

                float gx = std::abs(curr[c + 1] - curr[c - 1]) * 0.5f;
                float gy = std::abs(next[c] - prev[c]) * 0.5f;
                float g_norm = std::max(gx, gy) / d;

                if (g_norm > edge_gradient_threshold_) {
                    mask_row[c] = 255;
                }
            }
        }

        // Дилатация маски границ ядром 3x3 для полного удаления переходной полутени
        cv::dilate(edge_mask, edge_mask, cv::Mat(), cv::Point(-1, -1), 1);
        filtered_m.setTo(0.0f, edge_mask);
    }

    // 4. Подавление изолированных "летающих пикселей" (Flying Pixels)
    if (remove_flying_pixels_) {
        cv::Mat cleaned_m;
        filtered_m.copyTo(cleaned_m);

        int rows = filtered_m.rows;
        int cols = filtered_m.cols;

        for (int r = 1; r < rows - 1; ++r) {
            const float* prev_row = filtered_m.ptr<float>(r - 1);
            const float* curr_row = filtered_m.ptr<float>(r);
            const float* next_row = filtered_m.ptr<float>(r + 1);
            float* out_row = cleaned_m.ptr<float>(r);

            for (int c = 1; c < cols - 1; ++c) {
                float val = curr_row[c];
                if (val <= 0.0f) continue;

                // Подсчет связных соседей сходной глубины (< 15 см)
                int similar_neighbors = 0;
                float diff;

                diff = std::abs(prev_row[c] - val); if (prev_row[c] > 0.0f && diff < 0.15f) similar_neighbors++;
                diff = std::abs(next_row[c] - val); if (next_row[c] > 0.0f && diff < 0.15f) similar_neighbors++;
                diff = std::abs(curr_row[c - 1] - val); if (curr_row[c - 1] > 0.0f && diff < 0.15f) similar_neighbors++;
                diff = std::abs(curr_row[c + 1] - val); if (curr_row[c + 1] > 0.0f && diff < 0.15f) similar_neighbors++;

                // Если у пикселя нет связных соседей сходной глубины — удаляем артефакт
                if (similar_neighbors == 0) {
                    out_row[c] = 0.0f;
                }
            }
        }
        filtered_m = cleaned_m;
    }

    filtered_m.setTo(0.0f, ~valid_mask);

    // 5. Конвертация обратно в 16-битный формат с сохранением исходной шкалы
    cv::Mat result_mm;
    filtered_m.convertTo(result_mm, CV_16UC1, depth_scale_);
    return result_mm;
}

} // namespace core
} // namespace gtr1
