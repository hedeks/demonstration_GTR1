#pragma once

#include <opencv2/opencv.hpp>
#include <memory>

namespace gtr1 {
namespace core {

/**
 * @brief DepthFilter
 * Двусторонний (Bilateral) фильтр глубины с подавлением выбросов (Flying Pixels).
 * 1. Фильтрует гауссовский шум внутри гладких поверхностей (стол, пол, стены).
 * 2. Сохраняет резкие геометрические границы объектов без размытия краев.
 * 3. Удаляет изолированные летающие пиксели (Flying Pixels).
 */
class DepthFilter {
private:
    int diameter_{5};               // Диаметр окрестности пикселей (5x5)
    double sigma_depth_{0.04};      // Порог разрыва глубины (4 см)
    double sigma_space_{4.0};       // Пространственная сигма (пиксели)
    float min_range_m_{0.20f};      // Мин. дистанция
    float max_range_m_{5.00f};      // Макс. дистанция
    float depth_scale_{5000.0f};    // Шкала глубины (TUM RGB-D standard: 5000.0 единиц = 1.0 м)
    bool remove_flying_pixels_{true};
    bool remove_flying_edges_{true};       // Детекция и вырезание летающих граней (разрывов глубины)
    bool fill_small_holes_{true};          // Морфологическое заполнение микро-проколов (1-3 px)
    float edge_gradient_threshold_{0.08f}; // Порог градиента разрыва глубины

public:
    explicit DepthFilter(int diameter = 5,
                         double sigma_depth_m = 0.04,
                         double sigma_space_px = 4.0,
                         float min_range = 0.20f,
                         float max_range = 5.00f,
                         float depth_scale = 5000.0f);

    float get_depth_scale() const { return depth_scale_; }
    void set_depth_scale(float scale) { depth_scale_ = scale; }

    /**
     * @brief Применение двусторонней фильтрации к 16-битной карте глубины (CV_16UC1).
     * @param raw_depth_mm Входная карта глубины в миллиметрах (CV_16UC1)
     * @return Отфильтрованная карта глубины (CV_16UC1)
     */
    cv::Mat filter(const cv::Mat& raw_depth_mm) const;

    void set_parameters(int diameter, double sigma_depth_m, double sigma_space_px);
    void set_remove_flying_pixels(bool enable) { remove_flying_pixels_ = enable; }
    void set_remove_flying_edges(bool enable, float threshold = 0.08f) {
        remove_flying_edges_ = enable;
        edge_gradient_threshold_ = threshold;
    }
    void set_fill_small_holes(bool enable) { fill_small_holes_ = enable; }
    bool is_flying_edges_enabled() const { return remove_flying_edges_; }
    bool is_fill_small_holes_enabled() const { return fill_small_holes_; }
};

} // namespace core
} // namespace gtr1
