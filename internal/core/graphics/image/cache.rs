// Copyright © SixtyFPS GmbH <info@slint-ui.com>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-commercial

use std::collections::HashMap;

use super::{ImageInner, SharedImageBuffer, SharedPixelBuffer};
use crate::{slice::Slice, SharedString};

#[derive(PartialEq, Eq, Hash, Debug, derive_more::From)]
pub enum ImageCacheKey {
    Path(SharedString),
    EmbeddedData(by_address::ByAddress<&'static [u8]>),
}

// Cache used to avoid repeatedly decoding images from disk.
#[derive(Default)]
pub(crate) struct ImageCache(HashMap<ImageCacheKey, SharedImageBuffer>);

thread_local!(pub(crate) static IMAGE_CACHE: core::cell::RefCell<ImageCache>  = Default::default());

impl ImageCache {
    // Look up the given image cache key in the image cache and upgrade the weak reference to a strong one if found,
    // otherwise a new image is created/loaded from the given callback.
    fn lookup_image_in_cache_or_create(
        &mut self,
        cache_key: ImageCacheKey,
        image_create_fn: impl Fn() -> Option<image::DynamicImage>,
    ) -> Option<SharedImageBuffer> {
        Some(match self.0.entry(cache_key) {
            std::collections::hash_map::Entry::Occupied(existing_entry) => {
                existing_entry.get().clone()
            }
            std::collections::hash_map::Entry::Vacant(vacant_entry) => {
                let dynamic_image = image_create_fn()?;

                let shared_image_buffer = if dynamic_image.color().has_alpha() {
                    let rgba8image = dynamic_image.to_rgba8();
                    SharedImageBuffer::RGBA8(SharedPixelBuffer::clone_from_slice(
                        rgba8image.as_raw(),
                        rgba8image.width(),
                        rgba8image.height(),
                    ))
                } else {
                    let rgb8image = dynamic_image.to_rgb8();
                    SharedImageBuffer::RGB8(SharedPixelBuffer::clone_from_slice(
                        rgb8image.as_raw(),
                        rgb8image.width(),
                        rgb8image.height(),
                    ))
                };

                vacant_entry.insert(shared_image_buffer.clone());
                shared_image_buffer
            }
        })
    }

    pub(crate) fn load_image_from_path(&mut self, path: &SharedString) -> Option<ImageInner> {
        if path.is_empty() {
            return None;
        }
        let cache_key = ImageCacheKey::from(path.clone());
        self.lookup_image_in_cache_or_create(cache_key, || {
            image::open(std::path::Path::new(&path.as_str())).map_or_else(
                |decode_err| {
                    eprintln!("Error loading image from {}: {}", &path, decode_err);
                    None
                },
                Some,
            )
        })
        .map(|buffer| ImageInner::EmbeddedImage { path: path.clone(), buffer })
    }

    pub(crate) fn load_image_from_embedded_data(
        &mut self,
        data: Slice<'static, u8>,
        format: Slice<'static, u8>,
    ) -> Option<ImageInner> {
        let cache_key = ImageCacheKey::from(by_address::ByAddress(data.as_slice()));
        self.lookup_image_in_cache_or_create(cache_key, || {
            let format = std::str::from_utf8(format.as_slice())
                .ok()
                .and_then(image::ImageFormat::from_extension);
            let maybe_image = if let Some(format) = format {
                image::load_from_memory_with_format(data.as_slice(), format)
            } else {
                image::load_from_memory(data.as_slice())
            };

            match maybe_image {
                Ok(image) => Some(image),
                Err(decode_err) => {
                    eprintln!("Error decoding embedded image: {}", decode_err);
                    None
                }
            }
        })
        .map(|buffer| ImageInner::EmbeddedImage { path: Default::default(), buffer })
    }
}
