use proc_macro::TokenStream;
use quote::{quote, ToTokens};
use syn::{
    parse_macro_input, spanned::Spanned, Data, DeriveInput, Fields, Type
};

#[proc_macro_derive(VectorAlgebra)]
pub fn derive_vector_algebra(input: TokenStream) -> TokenStream {
    // Parse input
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    // Extract generics and prepare to extend where-clause
    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    // Ensure struct with single unnamed field
    let (elem_ty, len_expr) = match &input.data {
        Data::Struct(s) => match &s.fields {
            Fields::Unnamed(f) if f.unnamed.len() == 1 => {
                let field_ty = &f.unnamed.first().unwrap().ty;
                match field_ty {
                    Type::Array(arr) => {
                        (&arr.elem, &arr.len)
                    }
                    other => {
                        return syn::Error::new(
                            other.span(),
                            "VectorAlgebra expects a tuple struct with a single field of array type [T; N].",
                        )
                        .to_compile_error()
                        .into();
                    }
                }
            }
            _ => {
                return syn::Error::new(
                    input.span(),
                    "VectorAlgebra expects a tuple struct with a single unnamed field (a fixed-size array).",
                )
                .to_compile_error()
                .into();
            }
        },
        _ => {
            return syn::Error::new(input.span(), "VectorAlgebra only supports tuple structs.")
                .to_compile_error()
                .into();
        }
    };

    // Build added where-clause constraints: Elem: Copy + Add<Output=Elem> + Neg<Output=Elem>
    // We must keep existing where-clause and append our bounds.
    // Construct tokens for element type
    let elem_ty_tokens = elem_ty.to_token_stream();

    // We will create an augmented where clause by converting generics back to a `where` token stream.
    // First build the trait bounds we require.
    let added_bounds = quote! {
        #elem_ty_tokens: core::marker::Copy
            + core::ops::Add<Output = #elem_ty_tokens>
            + core::ops::Neg<Output = #elem_ty_tokens>
            + core::ops::Mul<Output = #elem_ty_tokens>
    };

    // Merge where clauses: if there is an existing where clause, append our added bounds.
    // We'll build a new where clause token stream.
    let where_tokens = if let Some(wc) = where_clause {
        // append to existing where clause
        quote! {
            #wc
            // additional bound
            , #added_bounds
        }
    } else {
        // create a new where clause
        quote! {
            where
                #added_bounds
        }
    };

    // Generate impls:
    // - Index and IndexMut for owned type (index into .0)
    // - Neg for &Type -> Type
    // - Add for &Type + &Type -> Type
    // - Sub for &Type - &Type -> Type implemented as self + (-rhs)
    //
    // Implementation approach: require Elem: Copy then use `let mut out = self.0;` and update elements.
    // Use the provided array length expression for loops and array initialization.

    let expanded = quote! {
        // preserve generics on impls
        impl #impl_generics core::ops::Index<usize> for #name #ty_generics
        #where_tokens
        {
            type Output = #elem_ty_tokens;
            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        impl #impl_generics core::ops::IndexMut<usize> for #name #ty_generics
        #where_tokens
        {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        impl<#impl_generics> core::ops::Neg for & #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn neg(self) -> Self::Output {
                // copy inner array (requires Copy on array, implied by Elem: Copy)
                let mut out = self.0;
                // update each element
                for i in 0..#len_expr {
                    out[i] = -self.0[i];
                }
                #name(out)
            }
        }

        impl<#impl_generics> core::ops::Add<& #name #ty_generics> for & #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn add(self, rhs: & #name #ty_generics) -> Self::Output {
                let mut out = self.0;
                for i in 0..#len_expr {
                    out[i] = self.0[i] + rhs.0[i];
                }
                #name(out)
            }
        }

        impl<#impl_generics> core::ops::Add<#name #ty_generics> for #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn add(self, rhs: #name #ty_generics) -> Self::Output {
                &self + &rhs
            }
        }

        impl<#impl_generics> core::ops::Add<& #name #ty_generics> for #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn add(self, rhs: & #name #ty_generics) -> Self::Output {
                &self + rhs
            }
        }

        impl<#impl_generics> core::ops::Add<#name #ty_generics> for & #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn add(self, rhs: #name #ty_generics) -> Self::Output {
                self + &rhs
            }
        }

        impl<#impl_generics> core::ops::Sub<& #name #ty_generics> for & #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn sub(self, rhs: & #name #ty_generics) -> Self::Output {
                // implement as self + (-rhs)
                // call Neg on rhs reference, which returns an owned value, then Add with &self
                self + (&-rhs)
            }
        }

        impl<#impl_generics> core::ops::Sub<#name #ty_generics> for #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn sub(self, rhs: #name #ty_generics) -> Self::Output {
                &self - &rhs
            }
        }

        impl<#impl_generics> core::ops::Sub<& #name #ty_generics> for #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn sub(self, rhs: & #name #ty_generics) -> Self::Output {
                &self - rhs
            }
        }

        impl<#impl_generics> core::ops::Sub<#name #ty_generics> for & #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn sub(self, rhs: #name #ty_generics) -> Self::Output {
                self - &rhs
            }
        }

        impl<#impl_generics> core::ops::Mul<#elem_ty_tokens> for & #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn mul(self, rhs: #elem_ty_tokens) -> Self::Output {
                let mut out = self.0;
                for i in 0..#len_expr {
                    out[i] = self.0[i] * rhs;
                }
                #name(out)
            }
        }

        impl<#impl_generics> core::ops::Mul<#elem_ty_tokens> for #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn mul(self, rhs: #elem_ty_tokens) -> Self::Output {
                &self * rhs
            }
        }

        impl<#impl_generics> core::ops::Div<#elem_ty_tokens> for & #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn div(self, rhs: #elem_ty_tokens) -> Self::Output {
                let mut out = self.0;
                for i in 0..#len_expr {
                    out[i] = self.0[i] / rhs;
                }
                #name(out)
            }
        }

        impl<#impl_generics> core::ops::Div<#elem_ty_tokens> for #name #ty_generics
        #where_tokens
        {
            type Output = #name #ty_generics;

            fn div(self, rhs: #elem_ty_tokens) -> Self::Output {
                &self / rhs
            }
        }
    };

    TokenStream::from(expanded)
}
